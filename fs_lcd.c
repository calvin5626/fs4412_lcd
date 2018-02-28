#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <asm/io.h> 
#include <linux/dma-mapping.h>

#define  HSPW    10
#define  HBPD    160
#define  HFPD    150
#define  VSPW    9
#define  VBPD    22
#define  VFPD    12

MODULE_LICENSE("GPL");
  
struct  fb_info  * fs_info;
unsigned  int  * gpf0con;
unsigned  int  * gpf1con;
unsigned  int  * gpf2con;
unsigned  int  * gpf3con;
unsigned  int  * gpd0con;
unsigned  int  * gpd0dat;
unsigned  int   *clk_div_lcd;
unsigned  int  * clk_src_lcd0;
unsigned  int  *  lcdblk_cfg;
unsigned  int  *lcdblk_cfg2;
unsigned  int  *  vidcon0;
unsigned  int  *  vidcon1;
unsigned  int  *  vidcon2;
unsigned  int  *  vidtcon0;
unsigned  int  *  vidtcon1;
unsigned  int  *  vidtcon2;
unsigned  int  * wincon0;
unsigned  int  *  vidoso0a;
unsigned  int  *  vidoso0b;
unsigned  int  *  vidoso0c;
unsigned   int  * shaadowcon;
unsigned   int  *  winchmap2;
unsigned  int   *  vidw00add0b0;
unsigned  int   *  vidw00add1b0;
unsigned  int  *win0map;//1
unsigned  int   *vidw00add2;//1




static unsigned int pseudo_palette[16];


static inline unsigned int chan_to_field(unsigned int chan,
					 struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}




static int fs_lcdfb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{

   unsigned  int  val;
  

if (regno < 16) {
		unsigned  int	*pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);

			pal[regno] = val;

		}
			return  0;
}

static struct fb_ops fs_lcdfb_ops = {
	.owner		= THIS_MODULE,
	
	.fb_setcolreg	= fs_lcdfb_setcolreg,
	
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

int  fs_lcd_init(void)
{
  int ret;
  unsigned  int  size  = PAGE_ALIGN(1024*600*2);
  dma_addr_t map_dma;
  
  //分配空间
  fs_info = framebuffer_alloc(0, NULL);
  if(!fs_info)
	{
		printk("failed to allocate framebuffer \n");
		return -ENOENT;
	}
 //标识lcd，使用字符串表示
  strcpy(fs_info->fix.id ,"fslcd");

   //显存的长度
  fs_info->fix.smem_len  = 1024*600*16/8*2;
  //像素值在显示内存中存储方式 说明最通用的方式—FB_TYPE_PACKED_PIXELS
  //方式。在这种方式下,像素值与 内存有直接对应关系
  fs_info->fix.type  =FB_TYPE_PACKED_PIXELS;
  //真彩屏
  fs_info->fix.visual =FB_VISUAL_TRUECOLOR;
  //一行的长度
  fs_info ->fix.line_length = 1024*2;
   //x方向的值
  fs_info->var.xres  = 1024 ;
  //y方向的值
  fs_info->var.yres  = 600;
  //x方向虚拟分辨率的值
  fs_info->var.xres_virtual  = 1024;
  //y方向虚拟分辨率的值
  fs_info->var.yres_virtual  = 600;
 
  //一个像素占多少位。一个像素由红，绿，蓝组成。使用888,565
  fs_info->var.bits_per_pixel  = 16;

  //红色在像素中占的长度
  fs_info->var.red.length  =5 ;
  //在像素中的偏移量
  fs_info->var.red.offset  = 11;
  fs_info->var.green.length  = 6;
  fs_info->var.green.offset  = 5;
  fs_info->var.blue.length  = 5;
  fs_info->var.blue.offset  =0;
  //立即生效
  fs_info->var.activate  = FB_ACTIVATE_NOW;
  //操作函数接口
  fs_info->fbops  = &fs_lcdfb_ops;
 
	 //调色板
	//void *pseudo_palette;		/* Fake palette of 16 colors */ 
  fs_info->pseudo_palette=pseudo_palette;
  fs_info->screen_size  = 1024*600*2*2; //显存的大小
 printk("fs_fb screen_size   = %ld\n",fs_info->screen_size);

  gpf0con   = ioremap(0x11400180,0x8);
  gpf1con   = ioremap(0x114001A0,0x4);
  gpf2con  =ioremap(0x114001C0,0x4);
  gpf3con  =ioremap(0x114001E0,0x4);
  gpd0con  = ioremap(0x114000A0,0x4);
  clk_div_lcd  = ioremap(0x1003c534,0x4);
  clk_src_lcd0 = ioremap(0x1003c234,0x4);
  lcdblk_cfg =  ioremap(0x10010210,0x4);
    lcdblk_cfg2  = ioremap(0x10010214,4);
  vidcon0  =  ioremap(0x11c00000,0x4);
  vidcon1  = ioremap(0x11c00004,0x4);
  vidcon2  =   ioremap(0x11c00008,0x4);
  vidtcon0  =  ioremap(0x11c00010,0x4);
  vidtcon1  =  ioremap(0x11c00014,0x4);
  vidtcon2  =  ioremap(0x11c00018,0x4);
  win0map = ioremap(0x11C00180,4);//1
  wincon0  = ioremap(0x11c00020,0x4);
  vidoso0a  = ioremap(0x11c00040,0x4);
   vidoso0b  = ioremap(0x11c00044,0x4);
   vidoso0c  = ioremap(0x11c00048,0x4);
   shaadowcon =  ioremap(0x11c00034,0x4);
   winchmap2  =  ioremap(0x11c0003c,0x4);
   vidw00add0b0  =  ioremap(0x11c000a0,0x4);
   vidw00add1b0  =  ioremap(0x11c000d0,0x4);
   vidw00add2 = ioremap(0x11C00104,4);//1
  *gpf0con  = 0x22222222;   //gpf0con   lcd  
  *gpf1con  = 0x22222222;
  *gpf2con  = 0x22222222;
  *gpf3con  = 0x00222222;
 
   
   *clk_div_lcd&= ~0xf;
   *clk_src_lcd0&= ~0xf;
   *clk_src_lcd0 |=6;  //选择时钟源
   *lcdblk_cfg|= 1 << 1;  //  fimd
      *lcdblk_cfg2|= 1;   //PWM  ENABLE
	printk("lcd  clk  \n");
	  *gpd0con  &= ~(0xf <<4) ;  //背光灯    gpd0_1  
	   *gpd0con  |=(0x1 <<4); //设置成输出模式
   gpd0dat  =  gpd0con  +1;  //输出高电平，背光灯亮 

   
  
   //CLKVAL  [13-6]  VCLK  = FIMD_CLK/CLKVAL+1   50M = 800M /CLKVAL+1                                 				                                                                                                                                                                                                                                                                                           15
   *vidcon0  = (15<<6); 
   *vidcon1 &= ~(1 << 7);//下降沿触发
   *vidcon1  |=((1 <<5)|(1 <<6)|(1 <<9));//参数4412  1748，lcd芯片手册，HYNC,VSYNC反转
   *vidcon2  = (1 <<14);  //14位必须设置成1NOTE: This bit should be set to 1.
    *vidtcon0  |= ((VBPD <<16)|(VFPD <<8)|(VSPW <<0));//参考lcd手册  p11
   *vidtcon1  |= ((HBPD <<16)|(HFPD <<8)|(HSPW <<0));
   *vidtcon2  |=((1023 <<0)|(599 << 11));//x方向，y方向的值
   *wincon0 &= ~(0xf << 2);
    //*wincon0  |=( (1<<16) | (5 <<2) |(0 <<9));
    *wincon0  |=( (1<<15) | (5 <<2) |(0 <<9));
	  *win0map = 0; //1
	 //映射，显存   screen_base  显存的虚拟地址，map_dma  物理地址
    fs_info->screen_base = dma_alloc_writecombine(NULL, size*2,  
							&map_dma, GFP_KERNEL);
  
	if(!fs_info->screen_base)
	{
		printk("dma_alloc_writecombine fail \n");
		return -ENOMEM;
	}
	  memset(fs_info->screen_base, 0x0, size);
	  fs_info->fix.smem_start = map_dma;  //显存的物理地址
  
     printk("dma alloc  \n");
  
	 *vidw00add0b0  = fs_info->fix.smem_start; //将显存的物理地址和lcd控制器关联在一起
	 *vidw00add1b0  = fs_info->fix.smem_start+fs_info->fix.smem_len;   //结束地址
      *vidw00add2 = (0 << 13) | ((1024 * 16 / 8 )<< 0);
   //*wincon0  |=( (1<<15) | (11 <<2) );//
  
   *vidoso0a |= ((0 <<11) |(0  <<0));  //左上角的坐标
   *vidoso0b |= ((1023 <<11) |(599  <<0));  //右下角的坐标
   *vidoso0c  |= (1024*600) ;//指定窗口的大小
  
 
	
 *shaadowcon  |=1;   //使能窗口0
  *gpd0dat  |=(1 <<1);  //点亮背光灯
   *vidcon0  |= (3 <<0); //使能
   *wincon0 |= (1 << 0);
 
 printk("register_framebuffer\n");
  
   ret = register_framebuffer(fs_info);
   if(ret < 0)
	{
		printk("failed to register framebuffer \n");
		return ret;
	}
   printk("fs_lcd_init  \n");
   
   return  0;
}
void  fs_lcd_exit(void)
{

	iounmap(gpf0con);
	iounmap(gpf1con);
	iounmap(gpf2con);
	iounmap(gpf3con);
      printk("fs_lcd_exit \n");
	  unregister_framebuffer(fs_info);
	  framebuffer_release(fs_info);
	  

}
module_init(fs_lcd_init);
module_exit(fs_lcd_exit);




