#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>// Module metadata
#include <linux/errno.h>

#include <linux/mutex.h>
#include <linux/hid.h>

#include <linux/serdev.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <uapi/linux/tty.h>

#include <linux/hwmon.h>

#ifndef KBUILD_MODNAME
#define KBUILD_MODNAME "cunt"
#endif

#define ARDUINOPWM_FANS 2
// TODO EXTEND RXBUF
#define ARDUINOPWM_SIZE_TXBUF 32
#define ARDUINOPWM_SIZE_RXBUF 32

// host -> arduino
// 0x00 0x01 0x01 0xA8 0x00
// |    |    |    |    L data
// |    |    |    L checksum
// |    |    L data length
// |    L command
// L start
enum arduinopwm_cmd : u8 {
	ARDUINOPWM_CMD_SETPWM=0x01, /* data: index u8, pwm value u8 */
	ARDUINOPWM_CMD_GETRPM=0x02, /* data: index u8 */
	ARDUINOPWM_CMD_GETVER=0x10, /* no data */
	ARDUINOPWM_CMD_SETREADINTERVAL=0x11, /* data: time in ms u16 */
};
// arduino -> host
// 0x00 0x01 0x01 0xA8 0x00
// |    |    |    |    L data bytes
// |    |    |    L checksum
// |    |    L data length
// |    L info type
// L start
enum arduinopwm_ack : u8 {
	ARDUINOPWM_ACK_SETPWM=0x01, /* data: index u8, pwm value u8 */
	ARDUINOPWM_ACK_GETRPM=0x02, /* data: index u8, rpm value u16 */
	ARDUINOPWM_ACK_GETVER=0x10, /* data: version u32 */
};

union arduinopwm_cmd_ack{
	enum arduinopwm_cmd cmd;
	enum arduinopwm_ack ack;
};

struct arduinopwm_packet {
	u8 startByte;
	union arduinopwm_cmd_ack ca;
	u8 dataLength;
	u8 checksum;
	u8 data[0];
};

static size_t headerSize = sizeof(struct arduinopwm_packet);

struct arduinopwm {
	char* name;
	struct hid_device *hdev;
	struct device* hwmon_dev;
	struct work_struct tx_work;
	u8 txbuf[ARDUINOPWM_SIZE_TXBUF];
	u8 rxbuf[ARDUINOPWM_SIZE_RXBUF];
	struct mutex mutex;
	enum ARDUINO_RX_STATES {
		AWAITING_HEADER,
		AWAITING_DATA,
	} state;

	struct arduinopwm_fan {
		int lastRPM;
		int lastPWM;
	} fans[ARDUINOPWM_FANS];
};

static const struct hwmon_chip_info arduinopwm_chip_info;

int txPacket(struct arduinopwm *apwm, struct arduinopwm_packet *pack)
{
	uint8_t packetLength = headerSize+pack->dataLength; // TODO check if correct
	//TODO set tx bytes left in apwm struct
	//TODO write tx worker to send the rest of the bytes
	int ret;
	pr_err("pack pointer %zu\n",(size_t)pack);
	pr_err("packet length %zu\n",(size_t)packetLength);

	ret = hid_hw_output_report(apwm->hdev, apwm->txbuf, packetLength);
	if (ret < 0)
		return ret;

	return 0;
}

int rxPacket(struct arduinopwm *apwm, struct arduinopwm_packet *pack)
{
	int channel=0;
	int pwmRPM=0;
	// check data with checksum
	switch(pack->ca.ack)
	{
		case ARDUINOPWM_ACK_GETVER:
			// TODO add null termination at the end of pack->data
			pr_err("Firmware version: %s\n", pack->data);
			break;
		case ARDUINOPWM_ACK_GETRPM:
				channel= *(u8*)pack->data;
				pwmRPM = *(u16*)&pack->data[1];
				apwm->fans[channel].lastRPM=pwmRPM;
				pr_err("ACK RPM: %d\n", pwmRPM);
			break;
		case ARDUINOPWM_ACK_SETPWM:
				channel= *(u8*)pack->data;
				pwmRPM = *(u8*)&pack->data[1];
				apwm->fans[channel].lastPWM=pwmRPM;
				pr_err("ACK PWM: %d\n", pwmRPM);
			break;
		default:
			{
				struct arduinopwm_packet* versionPacket = (void*)apwm->txbuf;
				versionPacket->startByte=0x00;
				versionPacket->ca.cmd=ARDUINOPWM_CMD_GETVER;
				versionPacket->dataLength=0x00;
				versionPacket->checksum=0x42;
				//txPacket(apwm, versionPacket);
			}
			pr_err("ACK not recognised: %d\n", pack->ca.ack);
			break;
	}
	return 0;
}

static int arduinopwm_hid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct arduinopwm *apwm;
	int ret;

	apwm = devm_kzalloc(&hdev->dev, sizeof(*apwm), GFP_KERNEL);
	if (!apwm)
		return -ENOMEM;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto out_hw_stop;

	apwm->hdev = hdev;
	hid_set_drvdata(hdev, apwm);
	mutex_init(&apwm->mutex);

	hid_device_io_start(hdev);

	apwm->hwmon_dev = hwmon_device_register_with_info(&hdev->dev,
													KBUILD_MODNAME,
													apwm,
													&arduinopwm_chip_info,
													0);
	if (IS_ERR(apwm->hwmon_dev)) {
		ret = PTR_ERR(apwm->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void arduinopwm_hid_remove(struct hid_device *hdev)
{
	struct arduinopwm *apwm = hid_get_drvdata(hdev);

	hwmon_device_unregister(apwm->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int arduinopwm_hid_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct arduinopwm *apwm = hid_get_drvdata(hdev);

	memcpy(&apwm->rxbuf, data, min(ARDUINOPWM_SIZE_RXBUF, size));
	struct arduinopwm_packet* rxPack = (struct arduinopwm_packet*)&apwm->rxbuf;
	rxPacket(apwm, rxPack);

	return 0;
}

static void arduinopwm_write_wakeup(struct tty_struct *tty)
{
	struct arduinopwm *apwm = tty->disc_data;
	if (apwm)
		schedule_work(&apwm->tx_work);
	else
		pr_err("slip_write_wakeup apwm is NULL :(\n");
}

static umode_t arudinopwm_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
	const struct arduinopwm *apwm = data; // TODO maybe not our local driver data Possibly not needed
	if(channel < 0 || channel >= ARDUINOPWM_FANS)
		return 0;

	if(type == hwmon_fan && attr == hwmon_fan_input)
		return 0444;
	if(type == hwmon_pwm && attr == hwmon_pwm_input)
		return 0644;

	return 0;
};

static int arduinopwm_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct arduinopwm *apwm = dev_get_drvdata(dev);
	if(channel<0 || channel >= ARDUINOPWM_FANS)
		return -ENOTSUPP;
	if(type==hwmon_fan && attr == hwmon_fan_input)
	{
		// Request RPM
		// TODO check if we're overwriting a packet that's actively being sent
		struct arduinopwm_packet *rpmRequest = (void*)apwm->txbuf;
		rpmRequest->startByte=0x00;
		rpmRequest->ca.cmd=ARDUINOPWM_CMD_GETRPM;
		rpmRequest->dataLength=1;
		rpmRequest->data[0] = channel;
		// te rog frumos bro...
		txPacket(apwm, rpmRequest);
		// return previous RPM
		*val = apwm->fans[channel].lastRPM;
		return 0;

	}
	if(type==hwmon_pwm && attr == hwmon_pwm_input)
	{
		*val = apwm->fans[channel].lastPWM;
		return 0;
	}
	return -ENOTSUPP;
}

static int arduinopwm_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long val)
{
	struct arduinopwm *apwm = dev_get_drvdata(dev);
	if(channel<0 || channel >= ARDUINOPWM_FANS)
		return -ENOTSUPP;
	if(val<0 || val > 255)
		return -EINVAL;
	if(type == hwmon_pwm && attr == hwmon_pwm_input)
	{
				// TODO check if we're overwriting a packet that's actively being sent
		struct arduinopwm_packet *pwmSet = (void*)apwm->txbuf;
		pwmSet->startByte=0x00;
		pwmSet->ca.cmd=ARDUINOPWM_CMD_GETRPM;
		pwmSet->dataLength=0;
		pwmSet->data[0] = channel;
		// te rog frumos bro... vreu sa ies cu motoru' :'(
		txPacket(apwm, pwmSet);
		return 0;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops arduinopwm_hwmon = {
	.is_visible	= arudinopwm_hwmon_is_visible,
	.read		= arduinopwm_hwmon_read,
	.write		= arduinopwm_hwmon_write,
};

static const struct hwmon_channel_info * const apwm_channel_info[] = {
	HWMON_CHANNEL_INFO(fan,
		HWMON_F_INPUT, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
		HWMON_PWM_INPUT, HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_chip_info arduinopwm_chip_info = {
	.ops = &arduinopwm_hwmon,
	.info = apwm_channel_info,
};

static const struct hid_device_id arduinopwm_devices[] = {
	{ HID_USB_DEVICE(0x8082, 0x8082) },
	{ }
};

static struct hid_driver arduinopwm_hid = {
	.name = KBUILD_MODNAME,
	.id_table = arduinopwm_devices,
	.probe = arduinopwm_hid_probe,
	.remove = arduinopwm_hid_remove,
	.raw_event = arduinopwm_hid_raw_event,
};

static int __init arduinopwm_init(void)
{
	pr_err("arduinopwm_init\n");
	int status;

	status = hid_register_driver(&arduinopwm_hid);
	if (status)
		pr_err("Can't register HID driver\n");

	return status;
}

static void __exit arduinopwm_exit(void)
{
	pr_err("arduinopwm_exit\n");

	hid_unregister_driver(&arduinopwm_hid);
}

module_init(arduinopwm_init);
module_exit(arduinopwm_exit);

MODULE_AUTHOR("Dan-Ștefan Dumitrof");
MODULE_DESCRIPTION("arduinopwm");
MODULE_LICENSE("GPL");// Custom init and exit methods
