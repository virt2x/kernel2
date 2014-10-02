#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/acpi.h>
#include <linux/freezer.h>
#include "tpm.h"

enum tis_access {
    TPM_ACCESS_VALID = 0x80,
    TPM_ACCESS_ACTIVE_LOCALITY = 0x20,
    TPM_ACCESS_REQUEST_PENDING = 0x04,
    TPM_ACCESS_REQUEST_USE = 0x02,
};

enum tis_status {
    TPM_STS_VALID = 0x80,
    TPM_STS_COMMAND_READY = 0x40,
    TPM_STS_GO = 0x20,
    TPM_STS_DATA_AVAIL = 0x10,
    TPM_STS_DATA_EXPECT = 0x08,
};

enum tis_int_flags {
    TPM_GLOBAL_INT_ENABLE = 0x80000000,
    TPM_INTF_BURST_COUNT_STATIC = 0x100,
    TPM_INTF_CMD_READY_INT = 0x080,
    TPM_INTF_INT_EDGE_FALLING = 0x040,
    TPM_INTF_INT_EDGE_RISING = 0x020,
    TPM_INTF_INT_LEVEL_LOW = 0x010,
    TPM_INTF_INT_LEVEL_HIGH = 0x008,
    TPM_INTF_LOCALITY_CHANGE_INT = 0x004,
    TPM_INTF_STS_VALID_INT = 0x002,
    TPM_INTF_DATA_AVAIL_INT = 0x001,
};

enum tis_defaults {
    TIS_MEM_BASE = 0xFED40000,
    TIS_MEM_LEN = 0x5000,
    TIS_SHORT_TIMEOUT = 750,    /* ms */
    TIS_LONG_TIMEOUT = 2000,    /* 2 sec */
};

#define    TPM_ACCESS(l)            (0x0000 | ((l) << 12))
#define    TPM_INT_ENABLE(l)        (0x0008 | ((l) << 12))
#define    TPM_INT_VECTOR(l)        (0x000C | ((l) << 12))
#define    TPM_INT_STATUS(l)        (0x0010 | ((l) << 12))
#define    TPM_INTF_CAPS(l)         (0x0014 | ((l) << 12))
#define    TPM_STS(l)               (0x0018 | ((l) << 12))
#define    TPM_DATA_FIFO(l)         (0x0024 | ((l) << 12))

#define    TPM_DID_VID(l)           (0x0F00 | ((l) << 12))
#define    TPM_RID(l)               (0x0F04 | ((l) << 12))

static LIST_HEAD(tis_chips);
static DEFINE_MUTEX(tis_lock);

/* Before we attempt to access the TPM we must see that the valid bit is set.
 * The specification says that this bit is 0 at reset and remains 0 until the
 * 'TPM has gone through its self test and initialization and has established
 * correct values in the other bits.' */
static int wait_startup(struct tpm_chip *chip, int l)
{
    unsigned long stop = jiffies + chip->vendor.timeout_a;
    do {
        if (ioread8(chip->vendor.iobase + TPM_ACCESS(l)) &
            TPM_ACCESS_VALID)
            return 0;
        msleep(TPM_TIMEOUT);
    } while (time_before(jiffies, stop));
    return -1;
}

static int check_locality(struct tpm_chip *chip, int l)
{
    if ((ioread8(chip->vendor.iobase + TPM_ACCESS(l)) &
         (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID)) ==
        (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID))
        return chip->vendor.locality = l;

    return -1;
}

static void release_locality(struct tpm_chip *chip, int l, int force)
{
    if (force || (ioread8(chip->vendor.iobase + TPM_ACCESS(l)) &
              (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) ==
        (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID))
        iowrite8(TPM_ACCESS_ACTIVE_LOCALITY,
             chip->vendor.iobase + TPM_ACCESS(l));
}

static int request_locality(struct tpm_chip *chip, int l)
{
    unsigned long stop, timeout;
    long rc;

    if (check_locality(chip, l) >= 0)
        return l;

    iowrite8(TPM_ACCESS_REQUEST_USE,
         chip->vendor.iobase + TPM_ACCESS(l));

    stop = jiffies + chip->vendor.timeout_a;

    if (chip->vendor.irq) {
again:
        timeout = stop - jiffies;
        if ((long)timeout <= 0)
            return -1;
        rc = wait_event_interruptible_timeout(chip->vendor.int_queue,
                              (check_locality
                               (chip, l) >= 0),
                              timeout);
        if (rc > 0)
            return l;
        if (rc == -ERESTARTSYS && freezing(current)) {
            clear_thread_flag(TIF_SIGPENDING);
            goto again;
        }
    } else {
        /* wait for burstcount */
        do {
            if (check_locality(chip, l) >= 0)
                return l;
            msleep(TPM_TIMEOUT);
        }
        while (time_before(jiffies, stop));
    }
    return -1;
}

static u8 tpm2_tis_status(struct tpm_chip *chip)
{
    return ioread8(chip->vendor.iobase +
               TPM_STS(chip->vendor.locality));
}

static void tpm2_tis_ready(struct tpm_chip *chip)
{
    /* this causes the current command to be aborted */
    iowrite8(TPM_STS_COMMAND_READY,
         chip->vendor.iobase + TPM_STS(chip->vendor.locality));
}

static int get_burstcount(struct tpm_chip *chip)
{
    unsigned long stop;
    int burstcnt;

    /* wait for burstcount */
    /* which timeout value, spec has 2 answers (c & d) */
    stop = jiffies + chip->vendor.timeout_d;
    do {
        burstcnt = ioread8(chip->vendor.iobase +
                   TPM_STS(chip->vendor.locality) + 1);
        burstcnt += ioread8(chip->vendor.iobase +
                    TPM_STS(chip->vendor.locality) +
                    2) << 8;
        if (burstcnt)
            return burstcnt;
        msleep(TPM_TIMEOUT);
    } while (time_before(jiffies, stop));
    return -EBUSY;
}

static int recv_data(struct tpm_chip *chip, u8 *buf, size_t count)
{
    int size = 0, burstcnt, i;
    while (size < count &&
           wait_for_tpm_stat(chip,
                 TPM_STS_DATA_AVAIL | TPM_STS_VALID,
                 chip->vendor.timeout_c,
                 &chip->vendor.read_queue, true)
           == 0) {
        burstcnt = get_burstcount(chip);
        for (; burstcnt > 0 && size < count; burstcnt--)
            buf[size++] = ioread8(chip->vendor.iobase +
                          TPM_DATA_FIFO(chip->vendor.
                                locality));
    }
    dev_info(chip->dev, "recv : >> \n");
    for(i=0; i<size; i++)
        dev_info(chip->dev, "\t buf[%d] : %08x \n", i, buf[i]);
    dev_info(chip->dev, "recv : << \n");
    return size;
}

static int tpm2_tis_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
    int size = 0;
    int expected, status;

    if (count < TPM_HEADER_SIZE) {
        size = -EIO;
        goto out;
    }

    /* read first 10 bytes, including tag, paramsize, and result */
    if ((size =
         recv_data(chip, buf, TPM_HEADER_SIZE)) < TPM_HEADER_SIZE) {
        dev_err(chip->dev, "Unable to read header\n");
        goto out;
    }

    expected = be32_to_cpu(*(__be32 *) (buf + 2));
    if (expected > count) {
        size = -EIO;
        goto out;
    }

    if ((size +=
         recv_data(chip, &buf[TPM_HEADER_SIZE],
               expected - TPM_HEADER_SIZE)) < expected) {
        dev_err(chip->dev, "Unable to read remainder of result\n");
        size = -ETIME;
        goto out;
    }

    wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
              &chip->vendor.int_queue, false);
    status = tpm2_tis_status(chip);
    if (status & TPM_STS_DATA_AVAIL) {    /* retry? */
        dev_err(chip->dev, "Error left over data\n");
        size = -EIO;
        goto out;
    }

out:
    tpm2_tis_ready(chip);
    release_locality(chip, chip->vendor.locality, 0);
    return size;
}

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int tpm2_tis_send_data(struct tpm_chip *chip, u8 *buf, size_t len)
{
    int rc, status, burstcnt;
    size_t count = 0;

    if (request_locality(chip, 0) < 0)
        return -EBUSY;

    status = tpm2_tis_status(chip);
    if ((status & TPM_STS_COMMAND_READY) == 0) {
        tpm2_tis_ready(chip);
        if (wait_for_tpm_stat
            (chip, TPM_STS_COMMAND_READY, chip->vendor.timeout_b,
             &chip->vendor.int_queue, false) < 0) {
            rc = -ETIME;
            goto out_err;
        }
    }

    while (count < len - 1) {
        burstcnt = get_burstcount(chip);
        for (; burstcnt > 0 && count < len - 1; burstcnt--) {
            iowrite8(buf[count], chip->vendor.iobase +
                 TPM_DATA_FIFO(chip->vendor.locality));
            count++;
        }

        wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
                  &chip->vendor.int_queue, false);
        status = tpm2_tis_status(chip);
        if ((status & TPM_STS_DATA_EXPECT) == 0) {
            rc = -EIO;
            goto out_err;
        }
    }

    /* write last byte */
    iowrite8(buf[count],
         chip->vendor.iobase + TPM_DATA_FIFO(chip->vendor.locality));
    wait_for_tpm_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c,
              &chip->vendor.int_queue, false);
    status = tpm2_tis_status(chip);
    if ((status & TPM_STS_DATA_EXPECT) != 0) {
        rc = -EIO;
        goto out_err;
    }

    return 0;

out_err:
    tpm2_tis_ready(chip);
    release_locality(chip, chip->vendor.locality, 0);
    return rc;
}

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int tpm2_tis_send(struct tpm_chip *chip, u8 *buf, size_t len)
{
    int rc;
    u32 ordinal;

    rc = tpm2_tis_send_data(chip, buf, len);
    if (rc < 0)
        return rc;
    /* go and do it */
    iowrite8(TPM_STS_GO,
         chip->vendor.iobase + TPM_STS(chip->vendor.locality));

    if (chip->vendor.irq) {
        ordinal = be32_to_cpu(*((__be32 *) (buf + 6)));
        if (wait_for_tpm_stat
            (chip, TPM_STS_DATA_AVAIL | TPM_STS_VALID,
             tpm_calc_ordinal_duration(chip, ordinal),
             &chip->vendor.read_queue, false) < 0) {
            rc = -ETIME;
            goto out_err;
        }
    }

    return len;
out_err:
    tpm2_tis_ready(chip);
    release_locality(chip, chip->vendor.locality, 0);
    return rc;
}

static bool tpm2_tis_req_canceled(struct tpm_chip *chip, u8 status)
{
    switch (chip->vendor.manufacturer_id) {
    case TPM_VID_WINBOND:
        return ((status == TPM_STS_VALID) ||
            (status == (TPM_STS_VALID | TPM_STS_COMMAND_READY)));
    case TPM_VID_STM:
        return (status == (TPM_STS_VALID | TPM_STS_COMMAND_READY));
    default:
        return (status == TPM_STS_COMMAND_READY);
    }
}

static const struct file_operations tis_ops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .open = tpm_open,
    .read = tpm_read,
    .write = tpm_write,
    .release = tpm_release,
};

static DEVICE_ATTR(pcrs, S_IRUGO, tpm2_show_pcrs, NULL);
static DEVICE_ATTR(ownerauth, S_IRUGO, tpm2_show_ownerauth, NULL);
static DEVICE_ATTR(endorseauth, S_IRUGO, tpm2_show_endorseauth, NULL);
static DEVICE_ATTR(phenable, S_IRUGO, tpm2_show_phenable, NULL);
static DEVICE_ATTR(shenable, S_IRUGO, tpm2_show_shenable, NULL);
static DEVICE_ATTR(ehenable, S_IRUGO, tpm2_show_ehenable, NULL);

static struct attribute *tis_attrs[] = {
    &dev_attr_pcrs.attr,
    &dev_attr_ownerauth.attr,
    &dev_attr_endorseauth.attr,
    &dev_attr_phenable.attr,
    &dev_attr_shenable.attr,
    &dev_attr_ehenable.attr,NULL,
};

static struct attribute_group tis_attr_grp = {
    .attrs = tis_attrs
};

static struct tpm_vendor_specific tpm2_tis = {
    .status = tpm2_tis_status,
    .recv = tpm2_tis_recv,
    .send = tpm2_tis_send,
    .cancel = tpm2_tis_ready,
    .req_complete_mask = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
    .req_complete_val = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
    .req_canceled = tpm2_tis_req_canceled,
    .attr_group = &tis_attr_grp,
    .miscdev = {
            .fops = &tis_ops,},
};

static irqreturn_t tis_int_probe(int irq, void *dev_id)
{
    struct tpm_chip *chip = dev_id;
    u32 interrupt;

    interrupt = ioread32(chip->vendor.iobase +
                 TPM_INT_STATUS(chip->vendor.locality));

    if (interrupt == 0)
        return IRQ_NONE;

    chip->vendor.probed_irq = irq;

    /* Clear interrupts handled with TPM_EOI */
    iowrite32(interrupt,
          chip->vendor.iobase +
          TPM_INT_STATUS(chip->vendor.locality));
    return IRQ_HANDLED;
}

static irqreturn_t tis_int_handler(int dummy, void *dev_id)
{
    struct tpm_chip *chip = dev_id;
    u32 interrupt;
    int i;

    interrupt = ioread32(chip->vendor.iobase +
                 TPM_INT_STATUS(chip->vendor.locality));

    if (interrupt == 0)
        return IRQ_NONE;

    if (interrupt & TPM_INTF_DATA_AVAIL_INT)
        wake_up_interruptible(&chip->vendor.read_queue);
    if (interrupt & TPM_INTF_LOCALITY_CHANGE_INT)
        for (i = 0; i < 5; i++)
            if (check_locality(chip, i) >= 0)
                break;
    if (interrupt &
        (TPM_INTF_LOCALITY_CHANGE_INT | TPM_INTF_STS_VALID_INT |
         TPM_INTF_CMD_READY_INT))
        wake_up_interruptible(&chip->vendor.int_queue);

    /* Clear interrupts handled with TPM_EOI */
    iowrite32(interrupt,
          chip->vendor.iobase +
          TPM_INT_STATUS(chip->vendor.locality));
    ioread32(chip->vendor.iobase + TPM_INT_STATUS(chip->vendor.locality));
    return IRQ_HANDLED;
}

static bool interrupts = true;
module_param(interrupts, bool, 0444);
MODULE_PARM_DESC(interrupts, "Enable interrupts");

static int tpm2_tis_init(struct device *dev, resource_size_t start,
            resource_size_t len, unsigned int irq)
{
    u32 vendor, intfcaps, intmask;
    int rc, i, irq_s, irq_e;
    struct tpm_chip *chip;

    printk("tpm2_tis_init() @@ \n");
    if (!(chip = tpm_register_hardware(dev, &tpm2_tis)))
        return -ENODEV;

    chip->vendor.iobase = ioremap(start, len);
    if (!chip->vendor.iobase) {
        rc = -EIO;
        goto out_err;
    }

    /* Default timeouts */
    chip->vendor.timeout_a = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
    chip->vendor.timeout_b = msecs_to_jiffies(TIS_LONG_TIMEOUT);
    chip->vendor.timeout_c = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
    chip->vendor.timeout_d = msecs_to_jiffies(TIS_SHORT_TIMEOUT);

    if (wait_startup(chip, 0) != 0) {
        rc = -ENODEV;
        goto out_err;
    }

    if (request_locality(chip, 0) != 0) {
        rc = -ENODEV;
        goto out_err;
    }

    vendor = ioread32(chip->vendor.iobase + TPM_DID_VID(0));
    chip->vendor.manufacturer_id = vendor;

    dev_info(dev,
         "2.0 TPM (device-id 0x%X, rev-id %d)\n",
         vendor >> 16, ioread8(chip->vendor.iobase + TPM_RID(0)));

    /* Figure out the capabilities */
    intfcaps =
        ioread32(chip->vendor.iobase +
             TPM_INTF_CAPS(chip->vendor.locality));
    dev_info(dev, "TPM interface capabilities @@ (0x%x):\n",
        intfcaps);
    if (intfcaps & TPM_INTF_BURST_COUNT_STATIC)
        dev_info(dev, "\tBurst Count Static\n");
    if (intfcaps & TPM_INTF_CMD_READY_INT)
        dev_info(dev, "\tCommand Ready Int Support\n");
    if (intfcaps & TPM_INTF_INT_EDGE_FALLING)
        dev_info(dev, "\tInterrupt Edge Falling\n");
    if (intfcaps & TPM_INTF_INT_EDGE_RISING)
        dev_info(dev, "\tInterrupt Edge Rising\n");
    if (intfcaps & TPM_INTF_INT_LEVEL_LOW)
        dev_info(dev, "\tInterrupt Level Low\n");
    if (intfcaps & TPM_INTF_INT_LEVEL_HIGH)
        dev_info(dev, "\tInterrupt Level High\n");
    if (intfcaps & TPM_INTF_LOCALITY_CHANGE_INT)
        dev_info(dev, "\tLocality Change Int Support\n");
    if (intfcaps & TPM_INTF_STS_VALID_INT)
        dev_info(dev, "\tSts Valid Int Support\n");
    if (intfcaps & TPM_INTF_DATA_AVAIL_INT)
        dev_info(dev, "\tData Avail Int Support\n");

    dev_info(dev, "\ttpm_do_selftest() ** \n");

    if (tpm2_do_selftest(chip)) {
        dev_err(dev, "TPM self test failed\n");
        rc = -ENODEV;
        goto out_err;
    }

    /* INTERRUPT Setup */
    init_waitqueue_head(&chip->vendor.read_queue);
    init_waitqueue_head(&chip->vendor.int_queue);

    intmask =
        ioread32(chip->vendor.iobase +
             TPM_INT_ENABLE(chip->vendor.locality));

    intmask |= TPM_INTF_CMD_READY_INT
        | TPM_INTF_LOCALITY_CHANGE_INT | TPM_INTF_DATA_AVAIL_INT
        | TPM_INTF_STS_VALID_INT;

    iowrite32(intmask,
          chip->vendor.iobase +
          TPM_INT_ENABLE(chip->vendor.locality));
    if (interrupts)
        chip->vendor.irq = irq;
    if (interrupts && !chip->vendor.irq) {
        irq_s =
            ioread8(chip->vendor.iobase +
                TPM_INT_VECTOR(chip->vendor.locality));
        if (irq_s) {
            irq_e = irq_s;
        } else {
            irq_s = 3;
            irq_e = 15;
        }

        for (i = irq_s; i <= irq_e && chip->vendor.irq == 0; i++) {
            iowrite8(i, chip->vendor.iobase +
                 TPM_INT_VECTOR(chip->vendor.locality));
            if (request_irq
                (i, tis_int_probe, IRQF_SHARED,
                 chip->vendor.miscdev.name, chip) != 0) {
                dev_info(chip->dev,
                     "Unable to request irq: %d for probe\n",
                     i);
                continue;
            }

            /* Clear all existing */
            iowrite32(ioread32
                  (chip->vendor.iobase +
                   TPM_INT_STATUS(chip->vendor.locality)),
                  chip->vendor.iobase +
                  TPM_INT_STATUS(chip->vendor.locality));

            /* Turn on */
            iowrite32(intmask | TPM_GLOBAL_INT_ENABLE,
                  chip->vendor.iobase +
                  TPM_INT_ENABLE(chip->vendor.locality));

            chip->vendor.probed_irq = 0;

            #if 0
            /* Generate Interrupts */
            tpm_gen_interrupt(chip);
            #endif
            chip->vendor.irq = chip->vendor.probed_irq;

            /* free_irq will call into tis_int_probe;
               clear all irqs we haven't seen while doing
               tpm_gen_interrupt */
            iowrite32(ioread32
                  (chip->vendor.iobase +
                   TPM_INT_STATUS(chip->vendor.locality)),
                  chip->vendor.iobase +
                  TPM_INT_STATUS(chip->vendor.locality));

            /* Turn off */
            iowrite32(intmask,
                  chip->vendor.iobase +
                  TPM_INT_ENABLE(chip->vendor.locality));
            free_irq(i, chip);
        }
    }
    if (chip->vendor.irq) {
        iowrite8(chip->vendor.irq,
             chip->vendor.iobase +
             TPM_INT_VECTOR(chip->vendor.locality));
        if (request_irq
            (chip->vendor.irq, tis_int_handler, IRQF_SHARED,
             chip->vendor.miscdev.name, chip) != 0) {
            dev_info(chip->dev,
                 "Unable to request irq: %d for use\n",
                 chip->vendor.irq);
            chip->vendor.irq = 0;
        } else {
            /* Clear all existing */
            iowrite32(ioread32
                  (chip->vendor.iobase +
                   TPM_INT_STATUS(chip->vendor.locality)),
                  chip->vendor.iobase +
                  TPM_INT_STATUS(chip->vendor.locality));

            /* Turn on */
            iowrite32(intmask | TPM_GLOBAL_INT_ENABLE,
                  chip->vendor.iobase +
                  TPM_INT_ENABLE(chip->vendor.locality));
        }
    }

    INIT_LIST_HEAD(&chip->vendor.list);
    mutex_lock(&tis_lock);
    list_add(&chip->vendor.list, &tis_chips);
    mutex_unlock(&tis_lock);


    return 0;
out_err:
    if (chip->vendor.iobase)
        iounmap(chip->vendor.iobase);
    tpm_remove_hardware(chip->dev);
    return rc;
}

static struct platform_driver tis_drv = {
    .driver = {
        .name       = "tpm2_tis",
        .owner        = THIS_MODULE,
    },
};

static struct platform_device *pdev;

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "Force device probe rather than using ACPI entry");
static int __init init_tis(void)
{
    int rc;

    rc = platform_driver_register(&tis_drv);
    if (rc < 0)
        return rc;
    if (IS_ERR(pdev=platform_device_register_simple("tpm2_tis", -1, NULL, 0)))
        return PTR_ERR(pdev);
    if((rc=tpm2_tis_init(&pdev->dev, TIS_MEM_BASE, TIS_MEM_LEN, 0)) != 0) {
        platform_device_unregister(pdev);
        platform_driver_unregister(&tis_drv);
    }

    return rc;
}

static void __exit cleanup_tis(void)
{
    struct tpm_vendor_specific *i, *j;
    struct tpm_chip *chip;
    mutex_lock(&tis_lock);
    list_for_each_entry_safe(i, j, &tis_chips, list) {
        chip = to_tpm_chip(i);
        tpm_remove_hardware(chip->dev);
        iowrite32(~TPM_GLOBAL_INT_ENABLE &
              ioread32(chip->vendor.iobase +
                   TPM_INT_ENABLE(chip->vendor.
                          locality)),
              chip->vendor.iobase +
              TPM_INT_ENABLE(chip->vendor.locality));
        release_locality(chip, chip->vendor.locality, 1);
        if (chip->vendor.irq)
            free_irq(chip->vendor.irq, chip);
        iounmap(i->iobase);
        list_del(&i->list);
    }
    mutex_unlock(&tis_lock);
    platform_device_unregister(pdev);
    platform_driver_unregister(&tis_drv);
}

module_init(init_tis);
module_exit(cleanup_tis);
MODULE_AUTHOR("Quan Xu (quan.xu@intel.com)");
MODULE_DESCRIPTION("TPM 2.0 Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
