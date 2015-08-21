/**
 * 
 * Implement USB portion of rtl8139 Network Card driver
 * Use this as a template. Add code in areas matching string "CODE HERE".  
 * In this phase of the project we will be writing USB routines. 
 * Compile the driver as module. Use the makefile provided earlier
 * Make sure to unload the production module:
 * 
 * # lsmod|grep rtl8150 
 * # rmmod rtl8150  
 * # lsmod
 * 
 * Load the driver and run "ifconfig -a", You should see the MAC 
 * Address read from the device memory. 
 *
 * Guidelines are provided to assist you with writing a usb portion of the 
 * driver. Do not limit yourself to it. You are encouraged to review source 
 * code of production driver. 
 */

#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>
#include <asm/uaccess.h>

//#define DRV_NAME "LDDA_USB"  // Use it to change name of interface from eth
static const char driver_name [] = "an-rtl8150";

/* Define these values to match your device */
#define	VENDOR_ID_REALTEK		0x0bda

#define PRODUCT_ID_RTL8150		0x8150
// rtl8150 Datasheet - Page 9, Vendor Specific Memory Read/Write Commands

#define RTL8150_REQT_READ       0xc0
#define RTL8150_REQT_WRITE      0x40

#define RTL8150_REQ_GET_REGS    0x05
#define RTL8150_REQ_SET_REGS    0x05

// Register offset in device memory  - rtl8150 Datasheet page 17 

#define	IDR	0x0120  	//  Device memory where MAC address is found



// Table of devices that work with this driver 
static struct usb_device_id rtl8150_table[] ={ 
  {USB_DEVICE(VENDOR_ID_REALTEK, PRODUCT_ID_RTL8150)},
  {}
};

/** 
 * This marks the usb_device_id table in the module image. This information 
 * loads the module on demand when the USBcard is inserted into  USB slot. 
 * It is part of Module auotload mechanism supported in Linux
 */
MODULE_DEVICE_TABLE(usb, rtl8150_table);

/* Device private structure */

struct rtl8150 {
  struct usb_device *udev; 	// USB device
  struct net_device *netdev;	// Net device
  struct net_device_stats stats;  // Net device stats
  spinlock_t lock;

  // Add rtl8150 device specific stuff later 
};

typedef struct rtl8150 rtl8150_t;

static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id);
static void rtl8150_disconnect(struct usb_interface *intf);
static int rtl8150_open(struct net_device *dev);
static int rtl8150_close(struct net_device *dev);
static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *dev);
static struct net_device_stats* rtl8150_get_stats(struct net_device *dev);


static struct usb_driver rtl8150_driver = {
  .name =	driver_name,
  .id_table =	rtl8150_table,
  .probe =	rtl8150_probe,
  .disconnect =	rtl8150_disconnect
};

static struct net_device_ops rtl8150_netdev_ops = {
  .ndo_open               = rtl8150_open,
  .ndo_stop               = rtl8150_close,
  .ndo_get_stats          = rtl8150_get_stats,
  .ndo_start_xmit         = rtl8150_start_xmit
};


/********* USB ROUTINES*************/
static int rtl8150_probe(struct usb_interface *intf,
                         const struct usb_device_id *id)
{
  struct net_device *net_dev;
  rtl8150_t *priv_dev;
  u8 mac[6];
  int ret;

  //Extract usb_device from the usb_interface structure 
  struct usb_device *udev = interface_to_usbdev(intf);

  printk(KERN_INFO "an_usb_rtl8150:probe called \n");

  /**
   * Linux Network Stack works with network device not the USB device. 
   * We need to allocate an ethernet network device. alloc_etherdev() 
   * allocates net device structure with memory allocated for 
   * rtl8150 private structure. Return ENOMEM if failed
   */
  /**
   * TODO use DRV_NAME
   * Set the device name to DRV_NAME instead of eth via memcpy  
   *
   * What routine provides access to device's private structure from the 
   * net_device instance 
   */
  //TODO alloc_netdev_mqs(sizeof_priv, "DRV_NAME%d", ether_setup, txqs, rxqs);
 // private data of rtl8150_t size alocated in net_dev
  net_dev = alloc_etherdev(sizeof(rtl8150_t));
  if (!net_dev)
    return -ENOMEM;

  //Get pointer to private data space in net_dev
  priv_dev = netdev_priv(net_dev);

  //Save ptrs to usb_device and net_dev
  priv_dev->udev = udev;	
  priv_dev->netdev = net_dev;
  
  //Initialize priv_dev spin_lock
  spin_lock_init(&priv_dev->lock);

  //Set net_dev operations
  net_dev->netdev_ops = &rtl8150_netdev_ops;

  //sysfs stuff. Sets up device link in /sys/class/net/interface_name 
  SET_NETDEV_DEV(net_dev, &intf->dev);

   /**
   * You can stuff device private structure into USB Interface 
   * structure using usb_set_intfdata. That can be retrieved later 
   * using usb_get_drvdata, for example, in disconnect or other driver 
   * functions
   */
  //Save pointer to priv_dev in interface's device->device_data
  usb_set_intfdata(intf, priv_dev);

  /* Register netdevice. If fail goto out  */
  if (register_netdev(net_dev) != 0) {
    printk(KERN_ALERT "error registering netdev\n");
    goto out;
  }	

  /**
   * Update net device struct with MAC and Broadcast address
   * RealTek8150 datasheet - page 17 states that IDR registers, IDR0-5, 
   * found at device memory location 0x0120 contains MAC Address of the
   * the NIC device
   */

  /** Fill net device netdev->dev_addr[6] and netdev->broadcast[6] 
   * arrays.  For broadcast address fill all octets with 0xff. 
   * 
   * In USB, you don't map device memory, instead you submit
   * control urb to USB core when needs to read/write device register memory. 
   * 
   * The structure that sends the control request to a USB device has to 
   * conform to USB specification (chapter 9) and is defined in 
   *	include/linux/usb/ch8.h
   *
   * struct usb_ctrlrequest {
   *	__u8 bRequestType;
   *	__u8 bRequest;
   *	__le16 mValue;
   *	__le16 wIndex;
   *	__le16 wLength;
   * } __attribute__ ((packed));
   * 
   * Read IDR memory location in device memory by submitting 
   * usb_control_msg() to USB core
   * int usb_control_msg(struct usb_device *dev, unsigned int pipe,
   * 		__u8 request, __u8 requesttype, __u16 value, 
   *		__u16 index, void *data, __u16 size, int timeout);
   * where: 
   *   - struct usb_device *dev: pointer to the target USB device
   *   - unsigned int pipe: endpoint of the target USB device where
   *	this message will be sent. This value is created  by calling
   *	usb_snd|usb_rcvctrlpipe
   *   - __u8 request: Requets value for the control message 
   *		- vendor specific (RTL8150_REQ_GET_REGS)
   *   - __u8 requesttype: Request type 
   - vendor specific (RTL8150_REQT_READ)
   *   - __u16 value: the USB message value for the control message
   *	   driver uses it to specify the memory location in 
   device memory such as IDR register location.
   *   - __u16 indx: Desired offset.  driver sets it to 0 
   *	            -  means start of the requested memory location 
   *   - void *data: A pointer to the data to send to the device if this
   *	 is an OUT endpoint. If this is an IN endpoint, this is a
   *	 pointer to where the data should be placed after being read
   *	 from the device. 
   *   - __u16 size: size of the buffer that is pointed to by the data
   *	 paramter above. It is a number of bytes to transfer
   *   - int timeout: The amount of time, in jiffies, that should be
   *	waited before timing out. If this value is 0, the function
   *	will wait forever for the message to complete. For USB compliant
   *	host, device requests with a data stage must start to return data
   *	in 500ms after the request.
   *
   *   Return Value: on success, it returns the number of bytes that
   *   are transferred to or from the device, othervise negative number
   *		
   *  Some parameters for usb_control_msg: request, requesttype, value, 
   * indx,size map directly to the USB specification for how a USB 
   * control message is defined.
   *   	  
   * usb_control_msg or usb_bulk_msg cannot be called from the 
   * interrupt context. Also, this function cannot be cancelled by
   * any other function, so be careful when using it; make sure that
   * your driver disconnect function knows enough to wait for the call
   * to complete before allowing itself to be unloaded from memory
   * 
   * Read six bytes into net_device structure member dev_addr from 
   * device memory location IDR 
   */ 

  // TODO obtained spin lock..
  ret = usb_control_msg(udev,                        // destination usb device
                  usb_rcvctrlpipe(udev, 0),         // receiving
                  RTL8150_REQ_GET_REGS,             // request
                  RTL8150_REQT_READ,                // request type
                  IDR,                              // idr - mac address.
                  0, 
                  &mac,sizeof(mac),500);                       // timeout ms

  printk(KERN_INFO "an_usb_rtl8150: usb control got back  %d bytes Mac[%pM] \n",ret,mac);
  net_dev->dev_addr = kmalloc(sizeof(mac),GFP_ATOMIC);
  memcpy(net_dev->dev_addr,mac,sizeof(mac));
  printk(KERN_INFO "an_usb_rtl8150: Mac Address : [%pM] \n",priv_dev->netdev->dev_addr);

  /* Length of Ethernet frame. It is a "hardware header length", number 
   * of octets that lead the transmitted packet before IP header, or 
   * other protocol information.  Value is 14 for Ethernet interfaces.
   */

  net_dev->hard_header_len = 14;

  return 0;

 out:
  usb_set_intfdata(intf, NULL);
  free_netdev(net_dev);
  return -EIO;
}

static int rtl8150_open(struct net_device *dev)
{
  printk(KERN_INFO "an-rtl8150_open \n");
  netif_start_queue(dev); /* transmission queue start */
  return 0;

}

static int rtl8150_close(struct net_device *dev)
{
  printk(KERN_INFO "an-rtl8150_close \n");
  netif_stop_queue(dev); /* transmission queue stop */
  return 0;

}

static int rtl8150_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
  printk(KERN_INFO "an-rtl8150_start_xmit \n");
  dev_kfree_skb(skb); /* Just free it for now */

  return 0;
	
}

static struct net_device_stats* rtl8150_get_stats(struct net_device *dev)
{
  struct rtl8150 *priv = netdev_priv( dev );

  printk("dev_get_stats: Add code later\n");

  /**
   * You cannot return NULL, make sure to return the address 
   * of net_dev_stat that is in device private structure
   */
  return &priv->stats;
}

/**
 */
static void rtl8150_disconnect(struct usb_interface *intf)
{

  rtl8150_t *dev = usb_get_intfdata(intf);
  if(dev) {
    unregister_netdev(dev->netdev);
    //    kfree(dev->intr_buff);
    free_netdev(dev->netdev);
  }
  /* Get address of device private structure */

  /* CODE HERE */
  //int priv;
  //  if (priv) {
	
    /**
     * Unregister and free memory of net_device structure
     * Call usb_set_intfdata(intf, NULL) to free memory	
     */

    /* CODE HERE */
//  }
}



module_usb_driver(rtl8150_driver);

MODULE_AUTHOR("Aakarsh Nair");
MODULE_DESCRIPTION("USB Driver for Realtek rtl8150 USB Ethernet Wired Card");
MODULE_LICENSE("Dual BSD/GPL");
