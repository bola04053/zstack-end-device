#include "variable.h"
#include "IC_w_r.h"
#include "MT_UART.h"
  uchar qq[4];
  uchar buf[4];

void Initial(void);
uchar IC_Test(void);
void IC_Init(void);


void Initial(void)
{
     CLKCONCMD &= ~0x40;                           //设置系统时钟源为32MHZ晶振 
    while(CLKCONSTA & 0x40);                      //等待晶振稳定为32M 
    CLKCONCMD &= ~0x47;                          //设置系统主时钟频率为32MHZ     

    
  /*  IC_CS P1_7 */
   P1DIR |= 1<<7;
   P1INP |= 1<<7;
   P1SEL &= ~(1<<7);
   
   /* IC_SCK  P0_1 */
   P0DIR |= 1<<1;
   P0INP |= 1<<1;
   P0SEL &= ~(1<<1);
   
      /* IC_MOSI P1_2 */
   P1DIR |= 1<<2;
   P1INP |= 1<<2;
   P1SEL &= ~(1<<2);
   
      /* IC_MISO P0_4 */
   P0DIR &= ~(1<<4);
   P0INP &= ~(1<<4);
   P0SEL &= ~(1<<4);
   
      /* IC_REST P0_5 */
   P0DIR |= 1<<5;
   P0INP |= 1<<5;
   P0SEL &= ~(1<<5);
  
  IC_SCK = 1;
  IC_CS = 1;
 // HalUARTWrite(0,"IC INIT SUCCESS!",16);
}


uchar IC_Test(void)
{
  //uint i;
  uchar find=0xaa;
  uchar ar;   
  ar = PcdRequest(0x52,buf);//寻卡
  if(ar != 0x26)
      ar = PcdRequest(0x52,buf);
  if(ar != 0x26)
      find = 0xaa;
   if((ar == 0x26)&&(find == 0xaa))
   {
   if(PcdAnticoll(qq) == 0x26);//防冲撞
   {
  // HalUARTWrite(0,qq,4);
   find = 0x00;
   return 1;
   }
   }
   return 0;
 
}

void IC_Init(void)
{
  Initial();
  PcdReset();
  M500PcdConfigISOType('A');
}