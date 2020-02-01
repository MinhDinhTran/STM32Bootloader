Trong quá trình phát triển ứng dụng thì yêu cầu update firmware cho sản phẩm là thực sự cần thiết. Không có ứng dụng nào có thể “hoàn thiện” ngay được cả. Việc cập nhật firmware có thể được dùng để khắc phục lỗi khi lỗi được phát hiện; dùng để nâng cấp, thêm tính năng cho sản phẩm… Nói chung mình thấy đây là một yêu cầu cần thiết. Mình thấy hiện nay những dòng vi điều khiển STM32 của hãng ST có USB thông dụng như STM32F103, STM32F042, STM32F072… thì có con đã được trang bị sẵn USB Bootloader khi xuất xưởng như dòng F0 còn dòng F1 thì cũng được trang bị Bootloader nhưng không phải USB Bootloader. Với dòng F0 như đề cập ở trên tuy được trang bị USB Bootloader nhưng không phải theo chuẩn HID, yêu cầu cần cài đặt Driver thì mới có thể dùng được và cần yêu cầu chuyển đổi file HEX qua định dạng DFU… Nói chung là sẽ gây khó khăn với một số không ít người dùng.

Vì vậy để tiện dụng nhất mình nghĩ nên tự viết Bootloader, sử dụng USB HID để có thể cắm là chạy và không cần cài đặt Driver.

Nói qua về Bootloader thì đơn giản đây là một chương trình chạy trên chip. Nó có nhiệm vụ giao tiếp với máy tính để lấy về firmware mới cần cập nhật, sau đó ghi firmware mới này vào flash của STM32 và thực thi firmware mới.

Project sẽ chia làm 3 phần:
* Phần 1: Chương trình Bootloader, đây là chương trình sẽ được chạy mỗi khi chip được cấp nguồn. Có nhiệm vụ kiểm tra xem người dùng có muốn vào chế độ cập nhật Firmware hay muốn chạy chương trình ứng dụng. Nếu người dùng muốn chạy chương trình ứng dụng thì nó sẽ chạy chương trình ứng dụng ngược lại nếu người dùng muốn cập nhật Firmware thì nó sẽ vào chế độ USB HID và chờ để nhận Firmware mới từ máy tính qua USB. Chương trình này cần được nạp vào chip 1 lần (có thể sử dụng mạch nạp như ST Link, Jlink…).
* Phần 2: Chương trình trên máy tính, đây là chương trình giao tiếp giữa máy tính với Bootloader. Gửi lệnh xóa, ghi…, gửi Firmware mới xuống mạch STM32.
* Phần 3: Chương trình ứng dụng mẫu. Do Bootloader đã chiếm một phần bộ nhớ Flash của STM32 nên cần điều chỉnh lại một số thông số như vector ngắt, bộ nhớ Flash… thì chương trình mới thực thi được bình thường. Chương trình này sẽ được cập nhật vào STM32 (STM32 đã nạp Bootloader) thông qua phần mềm ở phần 2, vì vậy có thể update chương trình cho STM32 mà không cần mạch nạp, có thể cập nhật cho những sản phẩm đã bán cho khách hàng.
-------------
- STM32F103C8T6 có 64KB bộ nhớ flash. Bắt đầu từ địa chỉ 0x8000000. Chương trình Bootloader sẽ chiếm 8KB flash đầu tiên
Firmware:
– Khi được cấp nguồn hoặc sau reset chương trình bootloader sẽ được thực thi. Sử dụng chân PA0 để kiểm tra xem sẽ thực thi chương trình ứng dụng hay vào chế độ cập nhật Firmware. Nếu chân PA0 ở mức 1 thì sẽ chạy ứng dụng. Nếu chân PA0 ở mức 0 thì sẽ vào chế độ cập nhật Firmware. Chương trình Bootloader sử dụng USB HID để tiện cho việc sử dụng, cắm là chạy không cần cài đặt Driver. Do Bootloader chiếm 8KB đầu tiên nên chương trình ứng dụng sẽ bắt đầu từ địa chỉ 0x8002000.
#define ApplicationAddress 0x08002000

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

GPIO_InitTypeDef gpioInit;
	
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
gpioInit.GPIO_Mode=GPIO_Mode_IPU;
gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
gpioInit.GPIO_Pin=GPIO_Pin_0;
GPIO_Init(GPIOA, &gpioInit);

if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
	JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);
	/* Initialize user application's Stack Pointer */
	__set_MSP(*(__IO uint32_t*) ApplicationAddress);
	Jump_To_Application();
}

Đoạn chương trình trên sẽ khởi tạo chân PA0 ở chế độ input – pull up. Nếu chân PA0 ở mức 1 thì sẽ chạy chương trình ứng dụng ngược lại thì sẽ vào chế độ cập nhật Firmware, sử dụng USB HID
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
gpioInit.GPIO_Mode=GPIO_Mode_Out_PP;
gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
gpioInit.GPIO_Pin=GPIO_Pin_13;
GPIO_Init(GPIOC, &gpioInit);
GPIO_SetBits(GPIOC, GPIO_Pin_13);
Set_System();

USB_Interrupts_Config();

Set_USBClock();

USB_Init();

while (1)
{
}

Định dạng dữ liệu truyền qua USB như sau:
Mỗi packet dài 64 byte. Byte đầu tiên của packet là byte ID. Giá trị byte đầu tiên này dùng để nhận dạng xem chương trình bootloader sẽ thực hiện công việc gì.
* 0x00: Xóa Flash
* 0x01: Ghi Flash
* 0x02: Đọc Flash
* 0x03: Khởi động lại chip

void EP1_OUT_Callback(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	
	USB_SIL_Read(EP1_OUT, rxBuff);
	switch(rxBuff[0]) {
	case 0:
		KT_Erase();
		break;
	case 1:
		KT_WriteFlash();
		break;
	case 2:
		KT_ReadFlash();
		break;
	case 3:
		KT_Reset();
		break;
	}
	
	SetEPRxStatus(ENDP1, EP_RX_VALID);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void KT_Erase(void) {
	//nhan gia tri va xoa page can thiet
	uint32_t u32Data;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	u32Data=rxBuff[4];
	u32Data<<=8;
	u32Data+=rxBuff[3];
	u32Data<<=8;
	u32Data+=rxBuff[2];
	u32Data<<=8;
	u32Data+=rxBuff[1];
	if(u32Data>=0x08002000) {
		FLASH_ErasePage(u32Data);
		__NOP();
		__NOP();
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

void KT_Reset(void) {
	/*
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	*/
	NVIC_SystemReset();
}

void KT_WriteFlash(void) {
	//ghi 1 DWORD
	uint32_t u32Data, u32Addr, i;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	u32Addr=rxBuff[4];
	u32Addr<<=8;
	u32Addr+=rxBuff[3];
	u32Addr<<=8;
	u32Addr+=rxBuff[2];
	u32Addr<<=8;
	u32Addr+=rxBuff[1];
	if(u32Addr>=0x08002000) {
		for(i=0; i<8; ++i) {
			u32Data=rxBuff[i*4+8];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+7];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+6];
			u32Data<<=8;
			u32Data+=rxBuff[i*4+5];
			
			FLASH_ProgramWord(u32Addr, u32Data);
			
			__NOP();
			__NOP();
			
			u32Addr+=4;
		}
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

void KT_ReadFlash(void) {
	//ghi 1 DWORD
	uint32_t u32Data, u32Addr, i;
	u32Addr=rxBuff[4];
	u32Addr<<=8;
	u32Addr+=rxBuff[3];
	u32Addr<<=8;
	u32Addr+=rxBuff[2];
	u32Addr<<=8;
	u32Addr+=rxBuff[1];
	if(u32Addr>=0x08002000) {
		for(i=0; i<16; ++i) {
			u32Data=*((uint32_t*)u32Addr);
			txBuff[i*4]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+1]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+2]=(uint8_t)u32Data;
			u32Data>>=8;
			txBuff[i*4+3]=(uint8_t)u32Data;
			u32Addr+=4;
		}
	}
	while(GetEPTxStatus(ENDP1)==EP_TX_VALID);
	USB_SIL_Write(EP1_IN, (uint8_t*) txBuff, 64);       
	SetEPTxValid(ENDP1);
}

----
Ở 2 phần trước mình đã giới thiệu sơ qua về cách hoạt động của Bootloader.
Phần này mình sẽ hướng dẫn cách cấu hình Project để sử dụng với Bootloader.

* Do bootloader chiếm 8KB nên chương trình ứng dụng sẽ bắt đầu ở địa chỉ 0x8002000 và dung lượng flash còn lại cho chương trình ứng dụng là 56KB vì vậy cần cấu hình trong Keil C như sau:
Tab Target
IROM1 0x8002000 0xE000
Trong file system_stm32f10x.c sửa VECT_TAB_OFFSET thành 0x2000
Thêm tùy chọn để Keil C sinh ra cả file BIN sau khi biên dịch:
fromelf --bin ".\out\@L.axf" --output ".\out\@L.bin"
Sử dụng phần mềm USB HID Bootloader để nạp file BIN mới vào STM32 qua cổng USB mà không cần mạch nạp:
* Nối chân PA0 với GND.
* Reset để đưa STM32 vào chế độ Bootloader
* Chạy phần mềm tìm đến file BIN
* Ấn chọn Update và chờ cập nhật xong

USB HID Bootloader STM32F103C8T6 - Firmware HID nạp vào chip
USB HID Bootloader STM32F103C8T6_Appz - App nạp HID
Blink_PC13_wBootloader - Firmware blink nạp qua app
