/*
* ---------------------------------------------------------------------------
* FILE: upcm.c
*
* PURPOSE:

* Copyright (C) 2012 by Cambridge Silicon Radio Ltd.
*
* Refer to LICENSE.txt included with this source code for details on
* the license terms.
*
* ---------------------------------------------------------------------------
*/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/extcon.h>
#include <linux/platform_device.h>

#include "upcm.h"

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <linux/delay.h>

#define UPCM_NAME	"upcm"

//#define ENABLE_UPCM_LOG

static struct extcon_dev  audio_extcon_dev;

#define UPCM_CARD_IDX  -1

static int index =  -1 ;  /* Index 0-MAX */
static char *id = UPCM_NAME;   /* ID for this card */
module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for Harmony driver.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for Harmony driver.");



struct snd_upcm_audio {
	struct snd_card *card;
	struct snd_upcm_stream *as;
	struct upcm_device * device;
};

struct snd_upcm_stream {
	struct snd_upcm_audio *audio_dev;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *pcm_substream;
	
	unsigned int hwptr_done;	/* processed byte position in the buffer */
	unsigned int transfer_done;		/* processed frames since last period update */
	spinlock_t lock;
};

#define UPCM_OUTPUT_BUFSIZE	4

#define MAX_BUFFER_SIZE  4096
struct upcm_device {
	struct mutex lock;
	spinlock_t qlock;
	bool trigger_started;
	bool file_opened;
	struct device *parent;
	struct snd_card *card;
	struct snd_upcm_audio* audio_dev;
	struct timer_list timer;
	__u8 input_buffer[MAX_BUFFER_SIZE];
};

static struct upcm_device upcm_dev;

static struct snd_pcm_hardware snd_mychip_capture_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,	// BE or LE?
	.rates =            SNDRV_PCM_RATE_16000,
	.rate_min =         16000,
	.rate_max =         16000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max =	1024 * 1024,
	.period_bytes_min =	64,
	.period_bytes_max =	512 * 1024,
	.periods_min =		2,
	.periods_max =		1024,
};

static int snd_u_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_upcm_stream *as = NULL;
	struct snd_pcm_runtime *runtime = NULL;
	mutex_lock(&upcm_dev.lock);
	as = substream->private_data;
	runtime = substream->runtime;
	runtime->hw = snd_mychip_capture_hw;
	as->pcm_substream = substream;
	mutex_unlock(&upcm_dev.lock);
	printk("UPCM : snd_u_capture_open\n");
	return 0;
}

static int snd_u_capture_close(struct snd_pcm_substream *substream)
{
	mutex_lock(&upcm_dev.lock);
	//substream->pcm->private_data = NULL;
	mutex_unlock(&upcm_dev.lock);
	printk("UPCM : snd_u_capture_close\n");
	return 0;
}

static int snd_u_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	//struct upcm_device *upcm = substream->pcm->private_data;
	unsigned int channels, rate, format, period_bytes, buffer_bytes;
	int ret = 0;

	format = params_format(hw_params);
	rate = params_rate(hw_params);
	channels = params_channels(hw_params);
	period_bytes = params_period_bytes(hw_params);
	buffer_bytes = params_buffer_bytes(hw_params);

	printk("UPCM : snd_u_hw_params ");
	printk("format %d, rate %d, channels %d, period_bytes %d, buffer_bytes %d\n",
		format, rate, channels, period_bytes, buffer_bytes);

	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
					       params_buffer_bytes(hw_params));

	format = params_format(hw_params);
	rate = params_rate(hw_params);
	channels = params_channels(hw_params);
	period_bytes = params_period_bytes(hw_params);
	buffer_bytes = params_buffer_bytes(hw_params);

	printk("UPCM: format 0x%x, rate %d, channels %d\n", format, rate, channels);
	
	if (ret < 0){
		printk("UPCM: snd_pcm_lib_alloc_vmalloc_buffer failed %d\n", ret);
	}
	
	return ret;
}

static int snd_u_hw_free(struct snd_pcm_substream *substream)
{
	int ret ; 
	mutex_lock(&upcm_dev.lock);
	ret = snd_pcm_lib_free_vmalloc_buffer(substream);
	mutex_unlock(&upcm_dev.lock);
	printk("UPCM : snd_u_hw_free\n");
	return ret;
}

static int snd_u_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_upcm_stream *as = NULL;
	mutex_lock(&upcm_dev.lock);
	as  = substream->private_data;
	as->hwptr_done = 0;
	as->transfer_done = 0;
	mutex_unlock(&upcm_dev.lock);
	printk("UPCM : snd_u_pcm_prepare\n");
	return 0;
}

static void start_idle_timer(void);
static void stop_idle_timer(void);

static  void trigger_start(bool start)
{
	spin_lock(&upcm_dev.qlock);
	upcm_dev.trigger_started = start;
	if (start && !upcm_dev.file_opened) {
		memset(upcm_dev.input_buffer, 0, sizeof(upcm_dev.input_buffer));
		start_idle_timer();
	}
	else if (!start)
		stop_idle_timer();
	spin_unlock(&upcm_dev.qlock);
}

static int snd_u_substream_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	printk("UPCM : snd_u_substream_capture_trigger, cmd %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		printk("UPCM: SNDRV_PCM_TRIGGER_START\n");
	///	mutex_lock(&upcm_dev.lock);
		trigger_start(true);
	//	mutex_unlock(&upcm_dev.lock);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		printk("UPCM: SNDRV_PCM_TRIGGER_STOP\n");
	//	mutex_lock(&upcm_dev.lock);
		trigger_start(false);
	//	mutex_unlock(&upcm_dev.lock);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		printk("UPCM: SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		printk("UPCM: SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
		break;
	}

	return 0;
}

static snd_pcm_uframes_t snd_u_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_upcm_stream *as = substream->private_data;
	unsigned int hwptr_done;

	#ifdef ENABLE_UPCM_LOG
	printk("UPCM : snd_u_pcm_pointer\n");
	#endif
	//mutex_lock(&upcm_dev.lock);
	spin_lock(&as->lock);
	hwptr_done = as->hwptr_done;
	spin_unlock(&as->lock);
	//mutex_unlock(&upcm_dev.lock);
	return hwptr_done / (substream->runtime->frame_bits >> 3);
}

static struct snd_pcm_ops snd_u_capture_ops = {
	.open =		snd_u_capture_open,
	.close =	snd_u_capture_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_u_hw_params,
	.hw_free =	snd_u_hw_free,
	.prepare =	snd_u_pcm_prepare,
	.trigger =	snd_u_substream_capture_trigger,
	.pointer =	snd_u_pcm_pointer,
	.page =		snd_pcm_lib_get_vmalloc_page,
	.mmap =		snd_pcm_lib_mmap_vmalloc,
};

static int snd_u_audio_dev_free(struct snd_device *device)
{
	struct snd_upcm_audio *audio_dev = device->device_data;

	kfree(audio_dev);
	
	return 0;
}

static void snd_u_audio_pcm_free(struct snd_pcm *pcm)
{
	struct snd_upcm_stream *stream = pcm->private_data;
	if (stream) {
		kfree(stream);
	}
}

static int snd_upcm_create_streams(struct snd_upcm_audio *audio_dev)
{
	struct snd_pcm *pcm;
	struct snd_upcm_stream *as;
	int ret;

	/* create a new pcm */
	as = kzalloc(sizeof(*as), GFP_KERNEL);
	if (!as)
		return -ENOMEM;
	
	as->audio_dev = audio_dev;
	
	ret = snd_pcm_new(audio_dev->card, "BLE Audio", 0, 0, 1, &pcm);
	if (ret < 0) {
		kfree(as);
		return ret;
	}
	
	as->pcm = pcm;
	pcm->private_data = as;
	pcm->private_free = snd_u_audio_pcm_free;
	pcm->info_flags = 0;
	strcpy(pcm->name, "BLE Audio");
	spin_lock_init(&as->lock);

	audio_dev->as = as;
		
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_u_capture_ops);

	return ret;
}


static int upcm_dev_input_lock(struct upcm_device *upcm, unsigned char * cp, unsigned int bytes)
{
	struct snd_upcm_stream *as = upcm->audio_dev->as;
	struct snd_pcm_runtime *runtime;
	unsigned int stride, frames, /*bytes,*/ oldptr;
	unsigned long flags;
	int period_elapsed = 0;

	spin_lock(&upcm_dev.qlock);
	if (!upcm_dev.trigger_started)
	{
		spin_unlock(&upcm_dev.qlock);
		return bytes;
	}
	spin_unlock(&upcm_dev.qlock);


	if(as->pcm_substream && as->pcm_substream->runtime && as->pcm_substream->runtime->dma_area) 
	{
		runtime = as->pcm_substream->runtime;
		stride = runtime->frame_bits >> 3;
		spin_lock_irqsave(&as->lock, flags);
		oldptr = as->hwptr_done;
		as->hwptr_done += bytes;
		if (as->hwptr_done >= runtime->buffer_size * stride)
			as->hwptr_done -= runtime->buffer_size * stride;
		frames = (bytes + (oldptr % stride)) / stride;
		as->transfer_done += frames;
		if (as->transfer_done >= runtime->period_size) {
			as->transfer_done -= runtime->period_size;
			period_elapsed = 1;
		}
		spin_unlock_irqrestore(&as->lock, flags);

		/* copy data */
		if (oldptr + bytes > runtime->buffer_size * stride) {
			unsigned int bytes1 =
					runtime->buffer_size * stride - oldptr;
			memcpy(runtime->dma_area + oldptr, cp, bytes1);
			memcpy(runtime->dma_area, cp + bytes1, bytes - bytes1);
		} else {
			memcpy(runtime->dma_area + oldptr, cp, bytes);
		}

		if (period_elapsed)
			snd_pcm_period_elapsed(as->pcm_substream);
	} else
		printk("UPCM: pcm_substream not allocated yet, discard!\n");
	
	return 0;
}

/********************************************************************************************************************
* UPCM Char Interface
*******************************************************************************************************************/


static void start_idle_timer()
{
	//printk("START IDLE TIMER HZ = %d\n", HZ); 
	upcm_dev.timer.expires = jiffies +  ( HZ * MAX_BUFFER_SIZE  / 2 / 16000 ) ;
	add_timer(&upcm_dev.timer);
}

static void stop_idle_timer()
{
	del_timer(&upcm_dev.timer);
}


static void upcm_idle_timer_function(unsigned long data) 
{
	spin_lock(&upcm_dev.qlock);
	if (!upcm_dev.trigger_started || upcm_dev.file_opened )
	{
		spin_unlock(&upcm_dev.qlock);
		return;
	}

	spin_unlock(&upcm_dev.qlock);
	start_idle_timer();
//	mutex_lock(&upcm_dev.lock);
//	memset(upcm_dev.input_buffer, 0, sizeof(upcm_dev.input_buffer));
	upcm_dev_input_lock(&upcm_dev, upcm_dev.input_buffer, sizeof(upcm_dev.input_buffer));
//	mutex_unlock(&upcm_dev.lock);
}


static int upcm_char_open(struct inode *inode, struct file *file)
{
	spin_lock(&upcm_dev.qlock); 
	if (upcm_dev.file_opened) 
	{
		spin_unlock(&upcm_dev.qlock);
		return -EBUSY;
	}
	upcm_dev.file_opened = true;
	stop_idle_timer();
	spin_unlock(&upcm_dev.qlock);
	
	file->private_data = &upcm_dev;
	nonseekable_open(inode, file);
	extcon_set_state(&audio_extcon_dev,EXTCON_MECHANICAL, 1);
	return 0;
}

static int upcm_char_release(struct inode *inode, struct file *file)
{
	printk("UPCM: upcm_char_release\n");
	file->private_data = NULL;
	extcon_set_state(&audio_extcon_dev, EXTCON_MECHANICAL,0);

	spin_lock(&upcm_dev.qlock);
	upcm_dev.file_opened = false;
	if (upcm_dev.trigger_started) {
		memset(upcm_dev.input_buffer, 0, sizeof(upcm_dev.input_buffer));
		start_idle_timer();
	}
	spin_unlock(&upcm_dev.qlock);

	return 0;
}

static ssize_t upcm_char_read(struct file *file, char __user *buffer,
				size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t upcm_char_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct upcm_device *dev = file->private_data;
	size_t len;

	len = min(sizeof(dev->input_buffer), count);
	if (copy_from_user(dev->input_buffer, buffer, len) )
		return -EFAULT;

	mutex_lock(&dev->lock);
	upcm_dev_input_lock(dev, dev->input_buffer, len);
	mutex_unlock(&dev->lock);
	return len;
}

static unsigned int upcm_char_poll(struct file *file, poll_table *wait)
{
	return -EOPNOTSUPP;
}


static const struct file_operations upcm_fops = {
	.owner		= THIS_MODULE,
	.open		= upcm_char_open,
	.release	= upcm_char_release,
	.read		= upcm_char_read,
	.write		= upcm_char_write,
	.poll		= upcm_char_poll,
	.llseek		= no_llseek,
};

static struct miscdevice upcm_misc = {
	.fops		= &upcm_fops,
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= UPCM_NAME,
};



static int upcm_dev_create(struct platform_device *pdev)
{
	struct snd_card *card = NULL;
	struct snd_upcm_audio *audio_dev = NULL;
	int ret;

	static struct snd_device_ops ops = {
		.dev_free =	snd_u_audio_dev_free,
	};

	memset(&upcm_dev, 0, sizeof(upcm_dev));
	mutex_init(&upcm_dev.lock);
	spin_lock_init(&upcm_dev.qlock);


	ret = snd_card_new(&pdev->dev, UPCM_CARD_IDX, "CSRUPCM", THIS_MODULE, sizeof(*audio_dev), &card);
	if (ret < 0) {
		printk("UPCM: snd_card_create failed %d\n", ret);
	//	goto unlock;
		return ret;
	}

	audio_dev = card->private_data;
	
	audio_dev->card = card;
	audio_dev->device = &upcm_dev;
	
	if ((ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, audio_dev, &ops)) < 0) {
		printk("UPCM: snd_device_new failed %d\n", ret);
		kfree(audio_dev);
		audio_dev = NULL;
		goto err_free;
	}

	if((ret = snd_upcm_create_streams(audio_dev)) < 0){
		printk("UPCM: snd_upcm_create_streams failed %d\n", ret);
		goto err_free;
	}

	if ((ret = snd_card_register(card)) < 0) {
		printk("UPCM: snd_card_register failed %d\n", ret);
		goto err_free;
	}

	upcm_dev.card = card;
	upcm_dev.audio_dev = audio_dev;	
	//upcm_dev.running = true;

	setup_timer(&upcm_dev.timer, upcm_idle_timer_function,
	             0); 


	printk("UPCM: upcm_dev_create successful\n");
	return 0;
err_free:
	snd_card_free(card);
	//upcm_dev.running = false;
	return ret;
}

static int __exit upcm_dev_destroy(void)
{
	int ret;
	mutex_lock(&upcm_dev.lock);
	ret = snd_card_free_when_closed(upcm_dev.card);
	if (ret < 0) {
		printk("UPCM: snd_card_free_when_closed failed %d\n", ret);
		//goto unlock;
		return ret;
	}

	mutex_unlock(&upcm_dev.lock);
	return ret;
}




static int cros_ec_lpc_probe(struct platform_device *pdev)
{
	upcm_dev_create(pdev);
	memset(&audio_extcon_dev, 0, sizeof(audio_extcon_dev));
	audio_extcon_dev.name = "ble_audio";
	extcon_dev_register(&audio_extcon_dev);
	return misc_register(&upcm_misc);
}

static int cros_ec_lpc_remove(struct platform_device *pdev)
{
	printk("UPCM : module exit\n");
	extcon_dev_unregister(&audio_extcon_dev);
	misc_deregister(&upcm_misc);
	upcm_dev_destroy();
    return 0;
}

static struct platform_driver cros_ec_lpc_driver = {
        .driver = {
                .name = UPCM_NAME,
        },
        .probe = cros_ec_lpc_probe,
        .remove = cros_ec_lpc_remove,
};

static struct platform_device cros_ec_lpc_device = {
        .name = UPCM_NAME
};

static int __init upcm_init(void)
{
	int ret;
	printk("UPCM : module init\n");

	ret = platform_driver_register(&cros_ec_lpc_driver);
	if (ret) {
			pr_err(UPCM_NAME ": can't register driver: %d\n", ret);
			return ret;
	}

	/* Register the device, and it'll get hooked up automatically */
	ret = platform_device_register(&cros_ec_lpc_device);
	if (ret) {
			pr_err(UPCM_NAME ": can't register device: %d\n", ret);
			platform_driver_unregister(&cros_ec_lpc_driver);
			return ret;
	}
	return 0;
}

static void  upcm_exit(void)
{
	platform_device_unregister(&cros_ec_lpc_device);
    platform_driver_unregister(&cros_ec_lpc_driver);
}

module_init(upcm_init);
module_exit(upcm_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cambridge Silicon Radio Ltd.");
MODULE_DESCRIPTION("ALSA Driver for HOG Audio");

