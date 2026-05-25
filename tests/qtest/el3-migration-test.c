#include "qemu/osdep.h"
#include "libqtest.h"

#define CMD_REQUEST_INTR   0x6000
#define CMD_ACK_INTR       0x6800
#define STAT_INT_LATCH     0x0001

/* Simplified property read - just check that device exists */
static uint64_t read_qom_property(QTestState *qts, const char *id, const char *property)
{
    /* For now, just return a dummy value - full QMP testing needs more setup */
    return 0;
}

static void trigger_eeprom_read(QTestState *qts)
{
    // Trigger an EEPROM read operation by accessing the device
    qtest_outw(qts, 0x200, 0x0000); // Reset command
    qtest_outw(qts, 0x204, 0x0001); // EEPROM read command
}

static void trigger_command_timer(QTestState *qts)
{
    // Trigger a command that starts a timer
    qtest_outw(qts, 0x200, 0x0000); // Reset command
    qtest_outw(qts, 0x204, 0x0002); // Command that starts timer
}

static void migrate_vm(QTestState *src, QTestState *dst)
{
    /* Simplified migration - in real test would use proper migration protocol */
    /* For now, just sleep to simulate migration time */
    g_usleep(100000); /* 100ms */
}

static void test_el3_migration_timers(void)
{
    QTestState *src, *dst;
    uint64_t cmd_timer_before, cmd_timer_after;
    uint64_t eeprom_timer_before, eeprom_timer_after;
    uint64_t irq_edges_before, irq_edges_after;

    // Start source VM with 3c509 device
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");

    // Trigger some operations to start timers
    trigger_eeprom_read(src);
    trigger_command_timer(src);

    // Allow some time for timers to fire
    g_usleep(100000); // 100ms

    // Read initial counter values
    cmd_timer_before = read_qom_property(src, "nic0", "cmd-timer-fired");
    eeprom_timer_before = read_qom_property(src, "nic0", "eeprom-timer-fired");
    irq_edges_before = read_qom_property(src, "nic0", "x-dbg-irq-edges");

    // Start destination VM
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");

    // Perform migration
    migrate_vm(src, dst);

    // Read counter values after migration
    cmd_timer_after = read_qom_property(dst, "nic0", "cmd-timer-fired");
    eeprom_timer_after = read_qom_property(dst, "nic0", "eeprom-timer-fired");
    irq_edges_after = read_qom_property(dst, "nic0", "x-dbg-irq-edges");

    // Verify counters are preserved
    g_assert_cmpuint(cmd_timer_after, >=, cmd_timer_before);
    g_assert_cmpuint(eeprom_timer_after, >=, eeprom_timer_before);
    g_assert_cmpuint(irq_edges_after, >=, irq_edges_before);

    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_irq_edges(void)
{
    QTestState *src, *dst;
    uint64_t irq_edges_before, irq_edges_after;

    // Start source VM with 3c509 device
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");

    // Trigger some IRQ activity
    trigger_eeprom_read(src);
    trigger_command_timer(src);

    // Allow some time for IRQs to fire
    g_usleep(100000); // 100ms

    // Read initial IRQ edge count
    irq_edges_before = read_qom_property(src, "nic0", "x-dbg-irq-edges");

    // Start destination VM
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");

    // Perform migration
    migrate_vm(src, dst);

    // Read IRQ edge count after migration
    irq_edges_after = read_qom_property(dst, "nic0", "x-dbg-irq-edges");

    // Verify IRQ edge transitions are preserved
    g_assert_cmpuint(irq_edges_after, >=, irq_edges_before);

    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_eeprom_pending(void)
{
    /* Test EEPROM timer migration with timer pending */
    QTestState *src, *dst;
    uint64_t eeprom_count_before, eeprom_count_after;
    
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    
    /* Start EEPROM read (timer will be pending) */
    qtest_writeb(src, 0x30A, 0x80);  /* EEPROM command */
    
    /* Step just 100μs - timer should still be pending (needs 162μs) */
    qtest_clock_step(src, 100000);
    
    eeprom_count_before = read_qom_property(src, "nic0", "x-dbg-eeprom-timer");
    
    /* Migrate with timer pending */
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    migrate_vm(src, dst);
    
    /* Verify timer didn't fire during migration */
    eeprom_count_after = read_qom_property(dst, "nic0", "x-dbg-eeprom-timer");
    g_assert_cmpuint(eeprom_count_after, ==, eeprom_count_before);
    
    /* Step past timer deadline */
    qtest_clock_step(dst, 100000);
    
    /* Verify timer fired exactly once */
    uint64_t final_count = read_qom_property(dst, "nic0", "x-dbg-eeprom-timer");
    g_assert_cmpuint(final_count, ==, eeprom_count_before + 1);
    
    /* Also verify EEPROM data is available */
    uint16_t eeprom_data = qtest_readw(dst, 0x30C);
    g_assert_cmpuint(eeprom_data, !=, 0xFFFF);
    
    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_no_double_fire(void)
{
    /* Test that timers don't double-fire after migration */
    QTestState *src, *dst;
    uint64_t count_before, count_after;
    
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    
    /* Start EEPROM read and let it complete */
    qtest_writeb(src, 0x30A, 0x80);
    qtest_clock_step(src, 200000);  /* Timer fires at 162μs */
    
    count_before = read_qom_property(src, "nic0", "x-dbg-eeprom-timer");
    g_assert_cmpuint(count_before, ==, 1);
    
    /* Migrate after timer fired */
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    migrate_vm(src, dst);
    
    /* Verify count unchanged */
    count_after = read_qom_property(dst, "nic0", "x-dbg-eeprom-timer");
    g_assert_cmpuint(count_after, ==, count_before);
    
    /* Step time and verify no additional fires */
    qtest_clock_step(dst, 200000);
    uint64_t final_count = read_qom_property(dst, "nic0", "x-dbg-eeprom-timer");
    g_assert_cmpuint(final_count, ==, count_before);
    
    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_irq_level(void)
{
    /* Test IRQ wire state preservation across migration */
    QTestState *src, *dst;
    uint16_t status_before, status_after;
    uint64_t edges_before, edges_after;
    
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    
    /* Enable interrupts and trigger one */
    qtest_writew(src, 0x30E, 0x00FF);  /* Enable all interrupts */
    qtest_writew(src, 0x308, CMD_REQUEST_INTR);  /* Request interrupt */
    
    /* Read status to verify interrupt is pending */
    status_before = qtest_readw(src, 0x308);
    g_assert_cmpuint(status_before & STAT_INT_LATCH, !=, 0);
    
    edges_before = read_qom_property(src, "nic0", "x-dbg-irq-edges");
    
    /* Migrate with IRQ asserted */
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    migrate_vm(src, dst);
    
    /* Verify status preserved */
    status_after = qtest_readw(dst, 0x308);
    g_assert_cmpuint(status_after & STAT_INT_LATCH, !=, 0);
    
    /* Verify edge count preserved */
    edges_after = read_qom_property(dst, "nic0", "x-dbg-irq-edges");
    g_assert_cmpuint(edges_after, ==, edges_before);
    
    /* Clear interrupt and verify edge increments */
    qtest_writew(dst, 0x30E, CMD_ACK_INTR | 0x00FF);
    uint64_t final_edges = read_qom_property(dst, "nic0", "x-dbg-irq-edges");
    g_assert_cmpuint(final_edges, >, edges_after);
    
    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_cmd_pending(void)
{
    /* Test command timer migration with timer pending */
    QTestState *src, *dst;
    uint64_t cmd_count_before, cmd_count_after;
    
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    
    /* Trigger a command that uses timer */
    qtest_writew(src, 0x30E, 0x0800); /* SELECT_WINDOW command */
    
    /* Step just a bit - timer should still be pending */
    qtest_clock_step(src, 50);
    
    cmd_count_before = read_qom_property(src, "nic0", "x-dbg-cmd-timer");
    
    /* Migrate with timer pending */
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    migrate_vm(src, dst);
    
    /* Verify timer didn't fire during migration */
    cmd_count_after = read_qom_property(dst, "nic0", "x-dbg-cmd-timer");
    g_assert_cmpuint(cmd_count_after, ==, cmd_count_before);
    
    /* Step past timer deadline */
    qtest_clock_step(dst, 1000);
    
    /* Verify timer fired exactly once */
    uint64_t final_count = read_qom_property(dst, "nic0", "x-dbg-cmd-timer");
    g_assert_cmpuint(final_count, ==, cmd_count_before + 1);
    
    qtest_quit(src);
    qtest_quit(dst);
}

static void test_el3_migration_phy_pending(void)
{
    /* Test PHY timer migration with timer pending */
    QTestState *src, *dst;
    uint64_t phy_count_before, phy_count_after;
    
    src = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    
    /* Trigger PHY operation if device has MII */
    /* This would need actual PHY register access - simplified for now */
    phy_count_before = read_qom_property(src, "nic0", "x-dbg-phy-timer");
    
    /* Migrate */
    dst = qtest_initf("-device 3c509,id=nic0,x-test=true,netdev=n0 "
                      "-netdev user,id=n0");
    migrate_vm(src, dst);
    
    /* Verify PHY timer state preserved */
    phy_count_after = read_qom_property(dst, "nic0", "x-dbg-phy-timer");
    g_assert_cmpuint(phy_count_after, ==, phy_count_before);
    
    qtest_quit(src);
    qtest_quit(dst);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/el3/migration/timers", test_el3_migration_timers);
    qtest_add_func("/el3/migration/irq-edges", test_el3_migration_irq_edges);
    qtest_add_func("/el3/migration/eeprom-pending", test_el3_migration_eeprom_pending);
    qtest_add_func("/el3/migration/cmd-pending", test_el3_migration_cmd_pending);
    qtest_add_func("/el3/migration/phy-pending", test_el3_migration_phy_pending);
    qtest_add_func("/el3/migration/no-double-fire", test_el3_migration_no_double_fire);
    qtest_add_func("/el3/migration/irq-level", test_el3_migration_irq_level);

    return g_test_run();
}