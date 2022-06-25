/* USB EHCI Host for Teensy 3.6
 * Copyright 2017 Paul Stoffregen (paul@pjrc.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Note: special thanks to the Linux kernel for the CH341's method of operation, particularly how the baud rate is encoded.
 */

#include <Arduino.h>
#include "USBHost_t36.h"  // Read this header first for key info
#include "CCID.h"

#define print   USBHost::print_
#define println USBHost::println_

//#define ENABLE_DEBUG_PINS

#ifdef ENABLE_DEBUG_PINS
#define debugDigitalToggle(pin)  {digitalWriteFast(pin, !digitalReadFast(pin));}
#define debugDigitalWrite(pin, state) {digitalWriteFast(pin, state);}
#else
#define debugDigitalToggle(pin)  {;}
#define debugDigitalWrite(pin, state) {;}
#endif

/************************************************************/
//  Define mapping VID/PID - to Serial Device type.
/************************************************************/
USBCCIDBase::product_vendor_mapping_t USBCCIDBase::pid_vid_mapping[] = {
	// FTDI mappings. 

	// AB Circle CIR315A NFC Reader
	{0x31AA, 0x3001, USBCCIDBase::CIR315A, 0},
};


/************************************************************/
//  Initialization and claiming of devices & interfaces
/************************************************************/

void USBCCIDBase::init()
{
	contribute_Pipes(mypipes, sizeof(mypipes)/sizeof(Pipe_t));
	contribute_Transfers(mytransfers, sizeof(mytransfers)/sizeof(Transfer_t));
	contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs)/sizeof(strbuf_t));
	driver_ready_for_device(this);
}

bool USBCCIDBase::claim(Device_t *dev, int claimtype, const uint8_t *descriptors, uint32_t len)
{
	const uint8_t *p = descriptors;
	const uint8_t *end = p + len;

	//---------------------------------------
	// Validate that this is a CCID Type of USB Device First!!
	// Validate the Vendor/Product First
	for (uint8_t i = 0; i < (sizeof(pid_vid_mapping)/sizeof(pid_vid_mapping[0])); i++) {
		if ((dev->idVendor == pid_vid_mapping[i].idVendor) && (dev->idProduct == pid_vid_mapping[i].idProduct)) {
			ccidtype = pid_vid_mapping[i].ccidtype;
			if (pid_vid_mapping[i].claim_at_type != claimtype) {
				println("CCID:Reject! - device wants to map at interface level");
				return false;
			}
			break;
		}
	}  

	if ((dev->bDeviceClass != 0) || (dev->bDeviceSubClass != 0)) { // Device Class and Sub Class should be both 0
		println("CCID:Reject! - Device Class and SubClass should be 0");
		return false;
	}

	bCardState = 0;
	
	println("\n>>CCID Claim Begin ======================");
	print("  Data Received>>");
	print_hexbytes(descriptors, len);
	println("  >>USBCCID claim this = ", (uint32_t)this, HEX);
	println("    >vid   = ", dev->idVendor, HEX);
	println("    >pid   = ", dev->idProduct, HEX);
	println("    >bDeviceClass    = ", dev->bDeviceClass);
   	println("    >bDeviceSubClass = ", dev->bDeviceSubClass);
	println("    >type            = ", claimtype);
   	println("    >bDeviceProtocol = ", dev->bDeviceProtocol);

	//---------------------------------------------------------------------------
	// Grab and Validate the Interface Descriptor Header
	const uint8_t bInterfaceLength              = p[0]; // 0
	const uint8_t bInterfaceDescriptorType      = p[1]; // 1
	//const uint8_t bInterfaceNumber              = p[2]; // 2
	//const uint8_t bInterfaceAlternateSetting    = p[3]; // 3
	//const uint8_t bInterfaceNumEndpoints        = p[4]; // 4
	const uint8_t bInterfaceClass               = p[5]; // 5
	const uint8_t bInterfaceSubClass            = p[6]; // 6
	const uint8_t bInterfaceProtocol            = p[7]; // 7
	//const uint8_t iInterfaceIndex               = p[8]; // 8

	// Validate that we have a CCID Header to work with
	if (bInterfaceLength != 0x09 || bInterfaceDescriptorType != 0x04 || bInterfaceClass != 0x0B || bInterfaceSubClass != 0x00 || bInterfaceProtocol != 0x00) {
		println("CCID:Reject! - Interface Header Incorrect");
		println(">>CCID Claim Done ======================");
		return false; // Interface descriptor
	}
	
	int rx_ep1     = 0;
	int tx_ep1     = 0;
	int rx_ep2     = 0;
	int tx_ep2     = 0;
	
	int rx_size1   = 0;
	int tx_size1   = 0;
	int rx_size2   = 0;
	int tx_size2   = 0;

	int intr_ep1    = 0;
	int intr_size1  = 0;

	int intr_ep2    = 0;
	int intr_size2  = 0;
	
	//---------------------------------------------------------------------------
	// Process Descriptors Passed By Device
	//---------------------------------------------------------------------------

	println("    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

	uint8_t interfaces = 0;
	uint8_t endpoints  = 0;

	while (p < end) {
		len = *p;
		if (len < 4) return false; 
		if (p + len > end) return false; // reject if beyond end of data

		uint32_t bLength = p[0];
		uint32_t bDescriptorType = p[1];

		// Section 4.3 Interface Descriptor, Contains the class, subclass and protocol
		if (bDescriptorType == 0x04 ) {
			//---------------------------------------------------------------------------
			// Grab and Validate the Interface Descriptor Header
			const uint8_t bInterfaceLength              = p[0]; // 0
			const uint8_t bInterfaceDescriptorType      = p[1]; // 1
			const uint8_t bInterfaceNumber              = p[2]; // 2
			const uint8_t bInterfaceAlternateSetting    = p[3]; // 3
			const uint8_t bInterfaceNumEndpoints        = p[4]; // 4
			const uint8_t bInterfaceClass               = p[5]; // 5
			const uint8_t bInterfaceSubClass            = p[6]; // 6
			const uint8_t bInterfaceProtocol            = p[7]; // 7
			const uint8_t iInterfaceIndex               = p[8]; // 8

			print  ("    Descriptor Type: INTERFACE ",bInterfaceNumber); println(", LEN:",bLength);
			println("       >bLength            = 0x", bInterfaceLength,HEX);
			println("       >bDescriptorType    = 0x", bInterfaceDescriptorType,HEX);
			println("     >>>bInterfaceNumber   = 0x", bInterfaceNumber,HEX);
			println("       >bAlternateSetting  = 0x", bInterfaceAlternateSetting,HEX);
			println("     >>>bNumEndpoints      = 0x", bInterfaceNumEndpoints,HEX);
			println("       >bInterfaceClass    = 0x", bInterfaceClass,HEX);
			println("       >bInterfaceSubClass = 0x", bInterfaceSubClass,HEX);
			println("       >bInterfaceProtocol = 0x", bInterfaceProtocol,HEX);
			println("       >iInterface         = 0x", iInterfaceIndex,HEX);
			print("       ");
			print_hexbytes(&p[0], bLength);

			// Make sure the fixed values match the CCID Specification
			if (bInterfaceLength            != 0x09 || 
			    bInterfaceDescriptorType    != 0x04 || 
				bInterfaceClass             != 0x0B || 
				bInterfaceSubClass          != 0x00 || 
				bInterfaceProtocol          != 0x00) {
				println("CCID:Reject! - Interface Header Incorrect");
				println(">>CCID Claim Done ======================");
				return false; // Interface descriptor
			}
			
			// Header Looks Good So far, lets keep processing....
			interfaces = bInterfaceNumber; 

		// Section 5.1 Descriptor - Smart Card Device Class Descriptor
		} else if (bDescriptorType == 0x21) {  // 0x21 = Functional Descriptor Type
			//---------------------------------------------------------------------------
			// Grab and Validate the Descriptor Header
			const uint8_t   bLength                = p[0];
			const uint8_t   bDescriptorType        = p[1];
			const uint16_t  bcdCCID                = (p[3] << 8) | p[2];
			const uint8_t   bMaxSlotIndex          = p[4];
			const uint8_t   bVoltageSupport        = p[5];
			const uint32_t  dwProtocols            = (p[9] << 24) | (p[8] << 16) | (p[7] << 8) | p[6];
			const uint32_t  dwDefaultClock         = (p[13] << 24) | (p[12] << 16) | (p[11] << 8) | p[10];
			const uint32_t  dwMaximumClock         = (p[17] << 24) | (p[16] << 16) | (p[15] << 8) | p[14];
			const uint8_t   bNumClockSupported     = p[18];
			const uint32_t  dwDataRate             = (p[22] << 24) | (p[21] << 16) | (p[20] << 8) | p[19];
			const uint32_t  dwMaxDataRate          = (p[26] << 24) | (p[25] << 16) | (p[24] << 8) | p[23];
			const uint8_t   bNumDataRatesSupported = p[27];
			const uint32_t  dwMaxIFSD              = (p[31] << 24) | (p[30] << 16) | (p[29] << 8) | p[28];
			const uint32_t  dwSynchProtocols       = (p[35] << 24) | (p[34] << 16) | (p[33] << 8) | p[32];
			const uint32_t  dwMechanical           = (p[39] << 24) | (p[38] << 16) | (p[37] << 8) | p[36];
			const uint32_t  dwFeatures             = (p[43] << 24) | (p[42] << 16) | (p[41] << 8) | p[40];
			const uint32_t  dwMaxCCIDMessageLength = (p[47] << 24) | (p[46] << 16) | (p[45] << 8) | p[44];
			const uint8_t   bClassGetResponse      = p[48];
			const uint8_t   bClassEnvelope         = p[49];
			const uint8_t   bLcdLayout_X           = p[50];
			const uint8_t   bLcdLayout_Y           = p[51];
			const uint8_t   bPINSupport            = p[52];
			const uint8_t   bMsxCCIDBusySlots      = p[53];

			println("    Descriptor Type: FUNCTIONAL");
			print  ("       >bLength                   = 0x", bLength, HEX); println(" >>Should Be: 0x36");
			print  ("       >bDescriptorType           = 0x", bDescriptorType, HEX); println(" >>Should Be: 0x21");
			print  ("       >bcdCCID                   = 0x", bcdCCID, HEX);  println(" >>Expected: 0x0110");
			print  ("       >bMaxSlotIndex             = 0x", bMaxSlotIndex, HEX);  println("");
			print  ("       >bVoltageSupport           = 0x", bVoltageSupport, HEX);  println("");
			print  ("       >dwProtocols               = 0x", dwProtocols, HEX);  println("");
			print  ("       >dwDefaultClock            = ", dwDefaultClock);  println("");
			print  ("       >dwMaximumClock            = ", dwMaximumClock);  println("");
			print  ("       >bNumClockSupported        = 0x", bNumClockSupported, HEX);  println("");
			print  ("       >dwDataRate                = ", dwDataRate);  println("");
			print  ("       >dwMaxDataRate             = ", dwMaxDataRate);  println("");
			print  ("       >bNumDataRatesSupported    = 0x", bNumDataRatesSupported, HEX);  println("");
			print  ("       >dwMaxIFSD                 = 0x", dwMaxIFSD, HEX);  println("");
			print  ("       >dwSynchProtocols          = 0x", dwSynchProtocols, HEX);  println("");
			print  ("       >dwMechanical              = 0x", dwMechanical, HEX);  println("");
			print  ("       >dwFeatures                = 0x", dwFeatures, HEX);  println("");
			print  ("       >dwMaxCCIDMessageLength    = ", dwMaxCCIDMessageLength);  println("");
			print  ("       >bClassGetResponse         = 0x", bClassGetResponse, HEX);  println("");
			print  ("       >bClassEnvelope            = 0x", bClassEnvelope, HEX);  println("");
			print  ("       >bLcdLayout_X              = 0x", bLcdLayout_X, HEX);  println("");
			print  ("       >bLcdLayout_Y              = 0x", bLcdLayout_Y, HEX);  println("");
			print  ("       >bPINSupport               = 0x", bPINSupport, HEX);  println("");
			print  ("       >bMsxCCIDBusySlots         = 0x", bMsxCCIDBusySlots, HEX);  println("");

			print("       ");
			print_hexbytes(&p[0], bLength);
			
			
		} else if (bDescriptorType == 0x05) {  // 0x05 = ENDPOINT Descriptor Type
			// endpoint descriptor
			print("    Descriptor Type: ENDPOINT ");
			print("Interface: ",interfaces);
			if (p[0] != 7) 
				return false; // 7 bytes
			if (p[3] == 2) {  // Bulk-out/Bulk-in Endpoints
				print(" Endpoint: ", p[2], HEX);
				switch (p[2] & 0xF0) {
				case 0x80:
					// IN endpoint
					if (rx_ep1 == 0 && interfaces == 0) {
						rx_ep1   = (p[2] & 0x0F);
						rx_size1 = (p[4] | (p[5] << 8));
						println(" Bulk-IN rx_size1 = ", rx_size1);
						print("       ");
						print_hexbytes(&p[0], bLength);
						endpoints++;
					}
					else if (rx_ep2 == 0 && interfaces == 1) {
						rx_ep2   = (p[2] & 0x0F);
						rx_size2 = (p[4] | (p[5] << 8));
						println(" Bulk-IN rx_size1 = ", rx_size2);
						print("       ");
						print_hexbytes(&p[0], bLength);
						endpoints++;
					} else {
						println(" NOT USED!!");
					}
					break;
				case 0x00:
					// OUT endpoint
					if (tx_ep1 == 0 && interfaces == 0) {
						tx_ep1   = (p[2]);
						tx_size1 = (p[4] | (p[5] << 8));
						println(" Bulk-OUT tx_size1 = ", tx_size1);
						print("       ");
						print_hexbytes(&p[0], bLength);
						endpoints++;
					}
					else if (tx_ep2 == 0) {
						tx_ep2   = (p[2]);
						tx_size2 = (p[4] | (p[5] << 8));
						println(" Bulk-OUT tx_size2 = ", tx_size2);
						print("       ");
						print_hexbytes(&p[0], bLength);
						endpoints++;
					} else {
						println(" NOT USED!!");
					}
					break;
				default:
					println("  invalid end point: ", p[2]);
					return false;
				}
			}
			if (p[3] == 3 && interfaces == 0) {  // Interrupt-IN Endpoints
				print(" Endpoint: ", p[2], HEX);
				if (intr_ep1 == 0 && interfaces == 0) {
					intr_ep1 = p[2] & 0x0F;
					intr_size1 = p[4] | (p[5] << 8);
					println(" Interrupt rx_size1 = ", intr_size1);
					print("       ");
					print_hexbytes(&p[0], bLength);
					endpoints++;
				}
				else if (intr_ep2 == 0) {
					intr_ep2 = p[2] & 0x0F;
					intr_size2 = p[4] | (p[5] << 8);
					println(" Interrupt rx_size2 = ", intr_size2);
					print("       ");
					print_hexbytes(&p[0], bLength);
					endpoints++;
				} else {
					println(" NOT USED!!");
				}
				endpoints++;
			}
		} else {
			println("  Unknown DescriptorType: ", bDescriptorType);
			return false; // unknown
		}
		p += len;
		println("    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
	}

	println("  Descriptors Processes.");
	println("  Interfaces Found: ",interfaces);
	println("  EndPoints Found: ",endpoints);

	print("  Bulk Endpoint");
	print(" rx1:",   rx_ep1);
    print(" tx1:",   tx_ep1);
	print(" rx2:",   rx_ep2);
    print(" tx2:",   tx_ep2);
    print(" intr1:", intr_ep1);
    print(" intr2:", intr_ep2);
	println();
	if (!rx_ep1 || !tx_ep1 || !rx_ep2 || !tx_ep2) return false; 	// did not get our two end points
	
	// Only going to Setup the First Interface
	if (!init_buffers1(rx_size1, tx_size1)) return false;
	if (!init_buffers2(rx_size2, tx_size2)) return false;

	println("rx1 buffer size:", rx_size1);
	println("tx1 buffer size:", tx_size1);
	println("rx2 buffer size:", rx_size2);
	println("tx2 buffer size:", tx_size2);
		
	rxpipe1 = new_Pipe(dev, 2, rx_ep1 & 15, 1, rx_size1);
	if (!rxpipe1) {
		 return false;
	}
	rxpipe1->callback_function = rx_callback1;
	queue_Data_Transfer(rxpipe1, rx1_1, rx_size1, this);

	txpipe1 = new_Pipe(dev, 2, tx_ep1, 0, tx_size1);
	if (!txpipe1) {
		 return false;
	}
	txpipe1->callback_function = tx_callback1;
	
	rxpipe2 = new_Pipe(dev, 2, rx_ep2 & 15, 1, rx_size2);
	if (!rxpipe2) {
		 return false;
	}
	rxpipe2->callback_function = rx_callback2;
	queue_Data_Transfer(rxpipe2, rx1_2, rx_size2, this);

	txpipe2 = new_Pipe(dev, 2, tx_ep2, 0, tx_size2);
	if (!txpipe2) {
		 return false;
	}
	txpipe2->callback_function = tx_callback2;

	// Setup the Interrupt Pipe to handle the two Interrupt Messages
	intrpipe1 = new_Pipe(dev, 3, intr_ep1, 1, 8, 0x0A);
	if (!intrpipe1) {
		 return false;
	}
	intrpipe1->callback_function = intr_callback1;
	queue_Data_Transfer(intrpipe1, intrdatabuff1, sizeof(intrdatabuff1), this);

	// Setup the Interrupt Pipe to handle the two Interrupt Messages
	intrpipe2 = new_Pipe(dev, 3, intr_ep2, 1, 8, 0x0A);
	if (!intrpipe2) {
		 return false;
	}
	intrpipe2->callback_function = intr_callback2;
	queue_Data_Transfer(intrpipe2, intrdatabuff2, sizeof(intrdatabuff2), this);
	
	control_queued = false;
	pending_control = 0x0;	// Maybe don't need to do...
	device = dev; // yes this is normally done on return from this but should not hurt if we do it here.

	println(">>CCID Claim Complete ======================");
	
	bSeq1 = 0;
	bSeq2 = 0;
	
	bCardState = NO_CARD;
	bCardNumberValid = 0;
	
	memset(bCardNumber,0,sizeof(bCardNumber));
	
	return true;

}

//=====================================================
void USBCCIDBase::intr_data1(const Transfer_t *transfer)
//=====================================================
{
	const uint8_t *p = (uint8_t *)transfer->buffer;
	uint32_t len = transfer->length;
	
	uint8_t bMessageType    = p[0];
	uint8_t bmSlotICCState  = p[1];
	
	if (bMessageType == 0x50) {
		//print("(CCID)<<Interrupt1: Got a RDR_to_PC_NotifySlotChange Message ");
		switch (bmSlotICCState) {
			case 0x02:	// Slot 0 Changed State and Slot is Empty
				print("Slot 0: Card Removed -> ");
				print_hexbytes(p,len);
				bCardState = NO_CARD;
				bCardNumberValid = 0;
				break;
			case 0x03:  // Slot 0 Changed State and Slot Has Detected Card
				bCardState = CARD_INSERTED;
				print("Slot 0: Card Inserted -> ");
				print_hexbytes(p,len);
				PC_to_RDR_IccPowerOff(txpipe1,0x00,bSeq1++);  // Parameters are Slot, Sequence
				//delay (400);
				break;
		}
	}
	
	//print("  >>Interrupt: ");
	//print_hexbytes(p,len);

	slot_state = bmSlotICCState;
	queue_Data_Transfer(intrpipe1, intrdatabuff1, sizeof(intrdatabuff1), this);
}

//======================================================
void USBCCIDBase::intr_data2(const Transfer_t *transfer)
//======================================================
{
	const uint8_t *p = (uint8_t *)transfer->buffer;
	uint32_t len = transfer->length;
	
	uint8_t bMessageType    = p[0];
	uint8_t bmSlotICCState  = p[1];
	
	if (bMessageType == 0x50) {
		print("(CCID)<<Interrupt2: Got a RDR_to_PC_NotifySlotChange Message ");
		switch (bmSlotICCState) {
			case 0x02:	// Slot 0 Changed State and Slot is Empty
				print("Slot 0: Card Removed -> ");
				print_hexbytes(p,len);
				bCardState = NO_CARD;
				bCardNumberValid = 0;
				break;
			case 0x03:  // Slot 0 Changed State and Slot Has Detected Card
				bCardState = CARD_INSERTED;
				print("Slot 0: Card Inserted -> ");
				print_hexbytes(p,len);
				PC_to_RDR_IccPowerOff(txpipe1,0x00,bSeq1++);  // Parameters are Slot, Sequence
				delay (400);
				break;
		}
	}
	
	//print("  >>Interrupt: ");
	//print_hexbytes(p,len);

	slot_state = bmSlotICCState;
	queue_Data_Transfer(intrpipe2, intrdatabuff2, sizeof(intrdatabuff2), this);
}


//====================================================
void USBCCIDBase::rx_data1(const Transfer_t *transfer)
//====================================================
{
	
	uint8_t *p = (uint8_t *)transfer->buffer;
	uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
	
	uint8_t bMessageType    = p[0];
	//uint8_t bmSlotICCState  = p[1];
	//uint32_t dwLength        = (p[4] << 24) | (p[3] << 16) | (p[2] << 8) | p[1];
	//uint8_t  bSlot           = p[5];
	uint8_t  bSeq			 = p[6];
	//uint8_t  bStatus		 = p[7];
	//uint8_t  bError			 = p[8];
	//uint8_t  bChainParameter = p[9];

	// Check the Bulk-In Messages

	switch (bMessageType) {
		case 0x80:	// RDR_to_PC_DataBlock
			print("(CCID)<<RDR_To_PC_DataBlock Received 1 -> ");
			print_hexbytes(p,len);


			if (bCardState == SND_XFRBLOCK) {
				bCardState = RPLY_XFRBLOCK;

				bCardLength = sizeof(bCardNumber);

				
				if (bCardType == NXPTAG_NTAG213) {
					bCardNumber[0] = p[10];
					bCardNumber[1] = p[11];
					bCardNumber[2] = p[12];
					bCardNumber[3] = p[14];
					bCardNumber[4] = p[15];
					bCardNumber[5] = p[16];
					bCardNumber[6] = p[17];
					bCardLength = 7;
				}

				if (bCardType == MiFare_Classic_1K) {
					bCardNumber[0] = p[10];
					bCardNumber[1] = p[11];
					bCardNumber[2] = p[12];
					bCardNumber[3] = p[13];
					bCardLength = 4;
				}
				
				print("Card ID: ");
				
				
				for (uint8_t i=0;i < bCardLength; i++){
					print("0x",bCardNumber[i],HEX);
					if (i < bCardLength-1) {
						print(",");
					} else {
						println();
					}
				}
				println("    Got Full Transaction from Card!!!");
				bCardState = CARD_ID_VALID;
			}

			if (bCardState == SND_POWER_ON) { 
				bCardState = RPLY_POWER_ON;
				
				// Grab our Card Type Here

				uint32_t dwLength        = (p[4] << 24) | (p[3] << 16) | (p[2] << 8) | p[1];
				uint8_t bCardType_A = p[11];
				uint8_t bCardType_B = 0;
				uint8_t bCardType_C = 0;
				if (dwLength == 0x14) {
					bCardType_B  = p[24];
					bCardType_C  = p[29];
				}
				
				print("Card Type: ",bCardType_A,HEX);
				print(", ",bCardType_B,HEX);
				print(", ",bCardType_C,HEX);
				println("");
				
				PrintTagType(bCardType_A, bCardType_B, bCardType_C);
				
				PC_to_RDR_SetParameters(txpipe1,0x00,bSeq++);	// Set Parameters of the Reader
			}
			break;
		case 0x81:	// RDR_to_PC_SlotStatus
			print("(CCID)<<RDR_To_PC_SlotStatus Received 1 -> ");
			print_hexbytes(p,len);
			if (bCardState == SND_POWER_OFF) {
				bCardState = RPLY_POWER_OFF;
				PC_to_RDR_IccPowerOn(txpipe1,0x00,bSeq++);  // Parameters are Slot, Sequence
			}
			break;
		case 0x82:	// RDR_to_PC_Parameters
			print("(CCID)<<RDR_To_PC_Parameters Received 1 -> ");
			print_hexbytes(p,len);
			if (bCardState == SND_SET_PARAMETERS) {
				bCardState = RPLY_SET_PARAMETERS;
				PC_to_RDR_XferBlock(txpipe1,0x00,bSeq++);  // Parameters are Slot, Sequence
			}
			break;
		case 0x83:	// RDR_to_PC_Escape
			print("(CCID)<<RDR_To_PC_Escape Received 1 -> ");
			print_hexbytes(p,len);
			break;
		case 0x84:	// RDR_to_PC_DataRateAndClockFrequence
			print("(CCID)<<RDR_To_PC_DataRateAndClockFrequence Received -> ");
			print_hexbytes(p,len);
			break;
		default:    // Unknown Message
			print("(CCID)<<Unknown Bulk-IN Message 1 -> ");
			print_hexbytes(p,len);
		    break;
	}
	
	queue_Data_Transfer(rxpipe1, rx1_1, rxsize1, this);
}

//====================================================
void USBCCIDBase::rx_data2(const Transfer_t *transfer)
//====================================================
{
	
	uint8_t *p2 = (uint8_t *)transfer->buffer;
	uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
	
	uint8_t bMessageType2    = p2[0];
	//uint8_t bmSlotICCState2  = p2[1];
	//uint32_t dwLength2        = (p2[4] << 24) | (p2[3] << 16) | (p2[2] << 8) | p2[1];
	//uint8_t  bSlot2           = p2[5];
	//uint8_t  bSeq2			 = p2[6];
	//uint8_t  bStatus2		 = p2[7];
	//uint8_t  bError2			 = p2[8];
	//uint8_t  bChainParameter2 = p2[9];

	// Check the Bulk-In Messages

	switch (bMessageType2) {
		case 0x80:	// RDR_to_PC_DataBlock
			print("(CCID)<<RDR_To_PC_DataBlock Received 2 -> ");
			print_hexbytes(p2,len);


			if (bCardState == SND_XFRBLOCK) {
				bCardState = RPLY_XFRBLOCK;
				
				bCardNumber[0] = p2[10];
				bCardNumber[1] = p2[11];
				bCardNumber[2] = p2[12];
				bCardNumber[3] = p2[14];
				bCardNumber[4] = p2[15];
				bCardNumber[5] = p2[16];
				bCardNumber[6] = p2[17];
				
				print("Card ID:");
				for (uint8_t i=0;i < sizeof(bCardNumber); i++){
					print("0x",bCardNumber[i],HEX);
					if (i < sizeof(bCardNumber)) {
						print(",");
					} else {
						println();
					}
				}
				
				println("Got Full Transaction from Card!!!");
			}

			if (bCardState == SND_POWER_ON) { 
				bCardState = RPLY_POWER_ON;
				PC_to_RDR_SetParameters(txpipe1,0x00,bSeq1++);	// Set Parameters of the Reader
			}
			break;
		case 0x81:	// RDR_to_PC_SlotStatus
			print("(CCID)<<RDR_To_PC_SlotStatus Received 2 -> ");
			print_hexbytes(p2,len);
			if (bCardState == SND_POWER_OFF) {
				bCardState = RPLY_POWER_OFF;
				PC_to_RDR_IccPowerOn(txpipe1,0x00,bSeq1++);  // Parameters are Slot, Sequence
			}
			break;
		case 0x82:	// RDR_to_PC_Parameters
			print("(CCID)<<RDR_To_PC_Parameters Received 2 -> ");
			print_hexbytes(p2,len);
			if (bCardState == SND_SET_PARAMETERS) {
				bCardState = RPLY_SET_PARAMETERS;
				PC_to_RDR_XferBlock(txpipe1,0x00,bSeq1++);  // Parameters are Slot, Sequence
			}
			break;
		case 0x83:	// RDR_to_PC_Escape
			print("(CCID)<<RDR_To_PC_Escape Received 2 -> ");
			print_hexbytes(p2,len);
			break;
		case 0x84:	// RDR_to_PC_DataRateAndClockFrequence
			print("(CCID)<<RDR_To_PC_DataRateAndClockFrequence Received -> ");
			print_hexbytes(p2,len);
			break;
		default:    // Unknown Message
			print("(CCID)<<Unknown Bulk-IN Message 2 -> ");
			print_hexbytes(p2,len);
		    break;
	}
	
	queue_Data_Transfer(rxpipe1, rx1_2, rxsize2, this);
}

//===============================================================================================
void USBCCIDBase::PC_to_RDR_GetSlotStatus(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence)
//===============================================================================================
{
	print("(CCID)>>PC_to_RDR_GetSlotStatus -> ");

	uint8_t Msg[] = { 0x65, 					// bMessageType
	                  0x00, 0x00, 0x00, 0x00,  // dwLength
				      0x00,                    // bSlot
					  0x00,                    // bSeq
					  0x00,0x00,0x00           // abRFU
				    };
							  
	// Set the Slot and Sequence number of the Command
	Msg[5] = slot;
	Msg[6] = sequence; 
	
	print_hexbytes(&Msg, sizeof(Msg) );
	
	queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);

	return;
}

//=============================================================================================
void USBCCIDBase::PC_to_RDR_IccPowerOff(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence)
//=============================================================================================
{
	print("(CCID)>>PC_to_RDR_IccPowerOff -> ");

	uint8_t Msg[] = { 0x63, 				   // bMessageType
	                  0x00, 0x00, 0x00, 0x00,  // dwLength
				      0x00,                    // bSlot
					  0x00,                    // bSeq
					  0x00,					   // Power Select - 0x00 - Automatic
					  0x00,0x00                // abRFU - Reserved for Future Use
				    };
							  
	// Set the Slot and Sequence number of the Command
	Msg[5] = slot;
	Msg[6] = sequence; 
	
	print_hexbytes(&Msg, sizeof(Msg) );
	
	queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);
	
	bCardState = SND_POWER_OFF;
	
	return;
}

//=============================================================================================
void USBCCIDBase::PC_to_RDR_IccPowerOn(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence)
//=============================================================================================
{
	print("(CCID)>>PC_to_RDR_IccPowerOn -> ");

	uint8_t Msg[] = { 0x62, 				   // bMessageType
	                  0x00, 0x00, 0x00, 0x00,  // dwLength
				      0x00,                    // bSlot
					  0x00,                    // bSeq
					  0x00,					   // Power Select - 0x00 - Automatic
					  0x00,0x00                // abRFU - Reserved for Future Use
				    };
							  
	// Set the Slot and Sequence number of the Command
	Msg[5] = slot;
	Msg[6] = sequence; 
	
	print_hexbytes(&Msg, sizeof(Msg) );
	
	queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);

	bCardState = SND_POWER_ON;
	
	return;
}

//===============================================================================================
void USBCCIDBase::PC_to_RDR_SetParameters(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence)
//===============================================================================================
{
	print("(CCID)>>PC_to_RDR_SetParameters -> ");

	uint8_t Msg[] = { 0x61, 				   		// bMessageType
	                  0x07, 0x00, 0x00, 0x00,  		// dwLength
				      0x00,                    		// bSlot
					  0x00,                    		// bSeq
					  0x01,					   		// bProtocolNum
					  0x00,0x00,               		// abRFU - Reserved for Future Use
					  // abProtocolDataStructure
					  0x11,							// bmFindexDindex
					  0x10,							// bmTCCKST1
					  0x00,							// bGardTimeT1
					  0x4D,							// bmWaitingIntegersT1
					  0x00,							// bClockStop
					  0x20,							// bIFSC
					  0x00							// bNadValue
				    };
							  
	// Set the Slot and Sequence number of the Command
	Msg[5] = slot;
	Msg[6] = sequence; 
	
	print_hexbytes(&Msg, sizeof(Msg) );
	
	queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);

	bCardState = SND_SET_PARAMETERS;
	
	return;
}

//===========================================================================================
void USBCCIDBase::PC_to_RDR_XferBlock(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence)
//===========================================================================================
{
	print("(CCID)>>PC_to_RDR_XferBlock -> ");

	if (bCardType == NXPTAG_NTAG213) {
		uint8_t Msg[] = { 0x6F, 				   // bMessageType
						  0x05, 0x00, 0x00, 0x00,  // dwLength
						  0x00,                    // bSlot
						  0x00,                    // bSeq
						  0x04,					   // Block Waiting Timeout
						  0x00, 0x00,              // Level Parameter
						  0xFF,                    // abData
						  0xB0,
						  0x00,
						  0x00,
						  0x10
						};
								  
		// Set the Slot and Sequence number of the Command
		Msg[5] = slot;
		Msg[6] = sequence; 
		
		print_hexbytes(&Msg, sizeof(Msg) );
		
		queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);
		bCardState = SND_XFRBLOCK;
	}
	
	if (bCardType == MiFare_Classic_1K) {
		uint8_t Msg[] = { 0x6F, 				   // bMessageType
						  0x05, 0x00, 0x00, 0x00,  // dwLength
						  0x00,                    // bSlot
						  0x00,                    // bSeq
						  0x04,					   // Block Waiting Timeout
						  0x00, 0x00,              // Level Parameter
						  0xFF,                    // abData
						  0xCA,
						  0x00,
						  0x00,
						  0x00
						};
								  
		// Set the Slot and Sequence number of the Command
		Msg[5] = slot;
		Msg[6] = sequence; 
		
		print_hexbytes(&Msg, sizeof(Msg) );
		
		queue_Data_Transfer(Bulk_Out_Pipe, Msg, sizeof(Msg), this);
		bCardState = SND_XFRBLOCK;
	}

	
	return;
}













// check if two legal endpoints, 1 receive & 1 transmit
bool USBCCIDBase::check_rxtx_ep1(uint32_t &rxep, uint32_t &txep)
{
	if ((rxep & 0x0F) == 0) return false;
	if ((txep & 0x0F) == 0) return false;
	uint32_t rxdir = rxep & 0xF0;
	uint32_t txdir = txep & 0xF0;
	if (rxdir == 0x80 && txdir == 0x00) {
		return true;
	}
	if (rxdir == 0x00 && txdir == 0x80) {
		std::swap(rxep, txep);
		return true;
	}
	return false;
}

bool USBCCIDBase::check_rxtx_ep2(uint32_t &rxep, uint32_t &txep)
{
	if ((rxep & 0x0F) == 0) return false;
	if ((txep & 0x0F) == 0) return false;
	uint32_t rxdir = rxep & 0xF0;
	uint32_t txdir = txep & 0xF0;
	if (rxdir == 0x80 && txdir == 0x00) {
		return true;
	}
	if (rxdir == 0x00 && txdir == 0x80) {
		std::swap(rxep, txep);
		return true;
	}
	return false;
}


// initialize buffer sizes and pointers
bool USBCCIDBase::init_buffers1(uint32_t rsize, uint32_t tsize)
{
	if (_big_buffer_size < (rsize + tsize) * 3 + 2) return false;
	rx1_1 = (uint8_t *)_bigBuffer;
	rx2_1 = rx1_1 + rsize;
	tx1_1 = rx2_1 + rsize;
	tx2_1 = tx1_1 + tsize;
	rxbuf_1 = tx2_1 + tsize;
	rxsize1 = ((_big_buffer_size/2) - (rsize + tsize) * 2) / 2;
	txsize1 = rxsize1;
	txbuf_1 = rxbuf_1 + rxsize1;
	rxhead1 = 0;
	rxtail1 = 0;
	txhead1 = 0;
	txtail1 = 0;
	rxstate1 = 0;
	return true;
}

// initialize buffer sizes and pointers
bool USBCCIDBase::init_buffers2(uint32_t rsize, uint32_t tsize)
{
	if (_big_buffer_size < (rsize + tsize) * 3 + 2) return false;
	rx1_2 = (uint8_t *)_bigBuffer + (_big_buffer_size/2);
	rx2_2 = rx1_1 + rsize;
	tx1_2 = rx2_1 + rsize;
	tx2_2 = tx1_1 + tsize;
	rxbuf_2 = tx2_1 + tsize;
	rxsize2 = ((_big_buffer_size/2) - (rsize + tsize) * 2) / 2;
	txsize2 = rxsize2;
	txbuf_2 = rxbuf_2 + rxsize2;
	rxhead2 = 0;
	rxtail2 = 0;
	txhead2 = 0;
	txtail2 = 0;
	rxstate2 = 0;
	return true;
}


void USBCCIDBase::disconnect()
{
	println("disconnect callback (CCID) ");

}


void USBCCIDBase::control(const Transfer_t *transfer)
{
	println("control callback (CCID) ", pending_control, HEX);
	control_queued = false;

	//mk_setup(setup, 0x40, 4, ftdi_format, 0, 0); // data format 8N1
	//queue_Control_Transfer(device, &setup, NULL, this);
	//control_queued = true;

	return;

}

/************************************************************/
//  Interrupt-based Data Movement
/************************************************************/

void USBCCIDBase::rx_callback1(const Transfer_t *transfer)
{
	//println("rx_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->rx_data1(transfer);
}

void USBCCIDBase::tx_callback1(const Transfer_t *transfer)
{
	//println("tx_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->tx_data1(transfer);
	//println("sent.");
}

void USBCCIDBase::rx_callback2(const Transfer_t *transfer)
{
	//println("rx_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->rx_data2(transfer);
}

void USBCCIDBase::tx_callback2(const Transfer_t *transfer)
{
	//println("tx_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->tx_data2(transfer);
	//println("sent.");
}

void USBCCIDBase::intr_callback1(const Transfer_t *transfer)
{
	//println("intr_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->intr_data1(transfer);
	//print("intr::callback: Moved Inbound Data\n");
}

void USBCCIDBase::intr_callback2(const Transfer_t *transfer)
{
	//println("intr_callback");
	if (!transfer->driver) return;
	((USBCCID *)(transfer->driver))->intr_data2(transfer);
	//print("intr::callback: Moved Inbound Data\n");
}



// re-queue packet buffer(s) if possible
void USBCCIDBase::rx_queue_packets1(uint32_t head, uint32_t tail)
{
	uint32_t avail;
	if (head >= tail) {
		avail = rxsize1 - 1 - head + tail;
	} else {
		avail = tail - head - 1;
	}
	uint32_t packetsize = rx2_1 - rx1_1;
	if (avail >= packetsize) {
		if ((rxstate1 & 0x01) == 0) {
			queue_Data_Transfer(rxpipe1, rx1_1, packetsize, this);
			rxstate1 |= 0x01;
		} else if ((rxstate1 & 0x02) == 0) {
			queue_Data_Transfer(rxpipe1, rx2_1, packetsize, this);
			rxstate1 |= 0x02;
		}
		if ((rxstate1 & 0x03) != 0x03 && avail >= packetsize * 2) {
			if ((rxstate1 & 0x01) == 0) {
				queue_Data_Transfer(rxpipe1, rx1_1, packetsize, this);
				rxstate1 |= 0x01;
			} else if ((rxstate1 & 0x02) == 0) {
				queue_Data_Transfer(rxpipe1, rx2_1, packetsize, this);
				rxstate1 |= 0x02;
			}
		}
	}
}

void USBCCIDBase::tx_data1(const Transfer_t *transfer)
{
	uint32_t mask;
	uint8_t *p = (uint8_t *)transfer->buffer;
	//debugDigitalWrite(5, HIGH);
	if (p == tx1_1) {
		println("tx1:");
		mask = 1;
		//txstate &= 0xFE;
	} else if (p == tx2_1) {
		println("tx2:");
		mask = 2;
		//txstate &= 0xFD;
	} else {
		debugDigitalWrite(5, LOW);
		return; // should never happen
	}
	// check how much more data remains in the transmit buffer
	uint32_t head = txhead1;
	uint32_t tail = txtail1;
	uint32_t count;
	if (head >= tail) {
		count = head - tail;
	} else {
		count = txsize1 + head - tail;
	}
	uint32_t packetsize = tx2_1 - tx1_1;
	// Only output full packets unless the flush bit was set.
	if ((count == 0) || ((count < packetsize) && ((txstate1 & 0x4) == 0) )) {
		// not enough data in buffer to fill a full packet
		txstate1 &= ~(mask | 4);	// turn off that transfer and make sure the flush bit is not set
		debugDigitalWrite(5, LOW);
		return;
	}
	// immediately transmit another full packet, if we have enough data
	if (count >= packetsize) count = packetsize;
	else txstate1 &= ~(mask | 4); // This packet will complete any outstanding flush

	println("TX:moar data!!!!");
	if (++tail >= txsize1) tail = 0;
	uint32_t n = txsize1 - tail;
	if (n > count) n = count;
	memcpy(p, txbuf_1 + tail, n);
	if (n >= count) {
		tail += n - 1;
		if (tail >= txsize1) tail = 0;
	} else {
		uint32_t len = count - n;
		memcpy(p + n, txbuf_1, len);
		tail = len - 1;
	}
	txtail1 = tail;
	queue_Data_Transfer(txpipe1, p, count, this);
	//debugDigitalWrite(5, LOW);
}

void USBCCIDBase::tx_data2(const Transfer_t *transfer)
{
	uint32_t mask;
	uint8_t *p = (uint8_t *)transfer->buffer;
	//debugDigitalWrite(5, HIGH);
	if (p == tx1_2) {
		println("tx1_2:");
		mask = 1;
		//txstate &= 0xFE;
	} else if (p == tx2_2) {
		println("tx2_2:");
		mask = 2;
		//txstate &= 0xFD;
	} else {
		debugDigitalWrite(5, LOW);
		return; // should never happen
	}
	// check how much more data remains in the transmit buffer
	uint32_t head = txhead2;
	uint32_t tail = txtail2;
	uint32_t count;
	if (head >= tail) {
		count = head - tail;
	} else {
		count = txsize2 + head - tail;
	}
	uint32_t packetsize = tx2_2 - tx1_2;
	// Only output full packets unless the flush bit was set.
	if ((count == 0) || ((count < packetsize) && ((txstate2 & 0x4) == 0) )) {
		// not enough data in buffer to fill a full packet
		txstate2 &= ~(mask | 4);	// turn off that transfer and make sure the flush bit is not set
		debugDigitalWrite(5, LOW);
		return;
	}
	// immediately transmit another full packet, if we have enough data
	if (count >= packetsize) count = packetsize;
	else txstate2 &= ~(mask | 4); // This packet will complete any outstanding flush

	println("TX2:moar data!!!!");
	if (++tail >= txsize2) tail = 0;
	uint32_t n = txsize2 - tail;
	if (n > count) n = count;
	memcpy(p, txbuf_2 + tail, n);
	if (n >= count) {
		tail += n - 1;
		if (tail >= txsize2) tail = 0;
	} else {
		uint32_t len = count - n;
		memcpy(p + n, txbuf_2, len);
		tail = len - 1;
	}
	txtail2 = tail;
	queue_Data_Transfer(txpipe2, p, count, this);
	//debugDigitalWrite(5, LOW);
}

void USBCCIDBase::flush1()
{
	print("USBCCIDBase::flush1");
 	if (txhead1 == txtail1) {
 		println(" - Empty");
 		return;  // empty.
 	}
 	//debugDigitalWrite(32, HIGH);
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	txtimer.stop();  		// Stop longer timer.
	txtimer.start(100);		// Start a mimimal timeout
//	timer_event(nullptr);   // Try calling direct - fails to work 
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	while (txstate1 & 3) ; // wait for all of the USB packets to be sent. 
	println(" completed");
 	//debugDigitalWrite(32, LOW);
}

void USBCCIDBase::flush2()
{
	print("USBCCIDBase::flush2");
 	if (txhead1 == txtail1) {
 		println(" - Empty");
 		return;  // empty.
 	}
 	//debugDigitalWrite(32, HIGH);
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	txtimer.stop();  		// Stop longer timer.
	txtimer.start(100);		// Start a mimimal timeout
//	timer_event(nullptr);   // Try calling direct - fails to work 
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	while (txstate1 & 3) ; // wait for all of the USB packets to be sent. 
	println(" completed");
 	//debugDigitalWrite(32, LOW);
}



void USBCCIDBase::timer_event(USBDriverTimer *whichTimer)
{
	debugDigitalWrite(7, HIGH);
	println("txtimer");
	uint32_t count;
	uint32_t head = txhead1;
	uint32_t tail = txtail1;
	if (head == tail) {
		println("  *** Empty ***");
		debugDigitalWrite(7, LOW);
		return; // nothing to transmit
	} else if (head > tail) {
		count = head - tail;
	} else {
		count = txsize1 + head - tail;
	}

	uint8_t *p;
	if ((txstate1 & 0x01) == 0) {
		p = tx1_1;
		txstate1 |= 0x01;
	} else if ((txstate1 & 0x02) == 0) {
		p = tx2_1;
		txstate1 |= 0x02;
	} else {
		txstate1 |= 4; 	// Tell the TX code to do flush code. 
		println(" *** No buffers ***");
		debugDigitalWrite(7, LOW);
		return; // no outgoing buffers available, try again later
	}

	uint32_t packetsize = tx2_1 - tx1_1;

	// Possible for remaining ? packet size and not have both? 
	if (count > packetsize) {
		txstate1 |= 4;	// One of the active transfers will handle the remaining parts
		count = packetsize;
	}

	if (++tail >= txsize1) tail = 0;
	uint32_t n = txsize1 - tail;
	if (n > count) n = count;
	memcpy(p, txbuf_1 + tail, n);
	if (n >= count) {
		tail += n - 1;
		if (tail >= txsize1) tail = 0;
	} else {
		uint32_t len = count - n;
		memcpy(p + n, txbuf_1, len);
		tail = len - 1;
	}
	txtail1 = tail;
	print("  TX data (", count);
	print(") ");
	print_hexbytes(p, count);
	queue_Data_Transfer(txpipe1, p, count, this);
	debugDigitalWrite(7, LOW);
}



/************************************************************/
//  User Functions - must disable USBHQ IRQ for EHCI access
/************************************************************/

void USBCCIDBase::begin(void)
{
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	//baudrate = baud;
	//bool format_changed = format != format_;
	//format_ = format; 
	//switch (sertype) {
	//	default:
	//	case CDCACM: pending_control |= 0x6; break;
	//	case FTDI: pending_control |= (format_changed? 0xf : 0xe); break;	// Set BAUD, FLOW, DTR
	//	case PL2303: pending_control |= 0x1e; break;  // set more stuff...
	//	case CH341: pending_control |= 0x1e; break;
	//	case CP210X: pending_control |= 0xf; break;
	//}
	if (!control_queued) control(NULL);
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	// Wait until all packets have been queued before we return to caller. 
	while (pending_control) {
		yield();	// not sure if we want to yield or what? 
	}
}

void USBCCIDBase::end(void)
{
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	//switch (sertype) {
	//	default:
	//	case CDCACM: pending_control |= 0x80; break;
	//	case FTDI: pending_control |= 0x80; break;	// clear DTR
	//	case PL2303: pending_control |= 0x80; break;
	//	case CH341: pending_control |= 0x80; break;
	//}
	if (!control_queued) control(NULL);
	NVIC_ENABLE_IRQ(IRQ_USBHS);

	// Wait until all packets have been queued before we return to caller. 
	while (pending_control) {
		yield();	// not sure if we want to yield or what? 
	}
}

int USBCCIDBase::available(void)
{
	if (!device) return 0;
	uint32_t head = rxhead1;
	uint32_t tail = rxtail1;
	if (head >= tail) return head - tail;
	return rxsize1 + head - tail;
}

int USBCCIDBase::peek(void)
{
	if (!device) return -1;
	if (rxhead1 == rxtail1) return -1;
	uint16_t tail = rxtail1 + 1;
	if (tail >= rxsize1) tail = 0;
	return rxbuf_1[tail];
}

int USBCCIDBase::read(void)
{
	if (!device) return -1;
	if (rxhead1 == rxtail1) return -1;
	if (++rxtail1 >= rxsize1) rxtail1 = 0;
	int c = rxbuf_1[rxtail1];
	if ((rxstate1 & 0x03) != 0x03) {
		NVIC_DISABLE_IRQ(IRQ_USBHS);
		rx_queue_packets1(rxhead1, rxtail1);
		NVIC_ENABLE_IRQ(IRQ_USBHS);
	}
	return c;
}

int USBCCIDBase::availableForWrite()
{
	if (!device) return 0;
	uint32_t head = txhead1;
	uint32_t tail = txtail1;
	if (head >= tail) return txsize1 - 1 - head + tail;
	return tail - head - 1;
}

size_t USBCCIDBase::write(uint8_t c)
{
	if (!device) return 0;
	uint32_t head = txhead1;
	if (++head >= txsize1) head = 0;
	while (txtail1 == head) {
		// wait...
	}
	txbuf_1[head] = c;
	txhead1 = head;
	//print("head=", head);
	//println(", tail=", txtail);

	// if full packet in buffer and tx packet ready, queue it
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	uint32_t tail = txtail1;
	if ((txstate1 & 0x03) != 0x03) {
		// at least one packet buffer is ready to transmit
		uint32_t count;
		if (head >= tail) {
			count = head - tail;
		} else {
			count = txsize1 + head - tail;
		}
		uint32_t packetsize = tx2_1 - tx1_1;
		if (count >= packetsize) {
			//println("txsize=", txsize);
			uint8_t *p;
			if ((txstate1 & 0x01) == 0) {
				p = tx1_1;
				txstate1 |= 0x01;
			} else /* if ((txstate & 0x02) == 0) */ {
				p = tx2_2;
				txstate1 |= 0x02;
			}
			// copy data to packet buffer
			if (++tail >= txsize1) tail = 0;
			uint32_t n = txsize1 - tail;
			if (n > packetsize) n = packetsize;
			//print("memcpy, offset=", tail);
			//println(", len=", n);
			memcpy(p, txbuf_1 + tail, n);
			if (n >= packetsize) {
				tail += n - 1;
				if (tail >= txsize1) tail = 0;
			} else {
				//n = txsize - n;
				uint32_t len = packetsize - n;
				//println("memcpy, offset=0, len=", len);
				memcpy(p + n, txbuf_1, len);
				tail = len - 1;
			}
			txtail1 = tail;
			//println("queue tx packet, newtail=", tail);
			debugDigitalWrite(7, HIGH);
			queue_Data_Transfer(txpipe1, p, packetsize, this);
			debugDigitalWrite(7, LOW);
			NVIC_ENABLE_IRQ(IRQ_USBHS);
			return 1;
		}
	}
	// otherwise, set a latency timer to later transmit partial packet
	txtimer.stop();
	txtimer.start(write_timeout_);
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	return 1;
}


uint8_t USBCCIDBase::GetSlotState(void)
{
	return slot_state;
}

uint8_t USBCCIDBase::GetCardState(void)
{
	return bCardState;
}

void USBCCIDBase::GetCardNumber(uint8_t *buff,uint8_t buffsize)
{
	for(uint8_t idx; idx < buffsize; idx++ && idx < sizeof(bCardNumber)) {
		buff[idx] = bCardNumber[idx];
	}

	return;
}

uint8_t USBCCIDBase::GetCardNumberLength()
{
	//println("Card Number Length: ",bCardLength);
	return bCardLength;
}

void USBCCIDBase::PrintTagType(uint8_t ID1, uint8_t ID2, uint8_t ID3)
{
		if (ID1 == 0x8F && ID2 == 0x26 && ID3 == 0x4D) {
			bCardType = HOTEL1;
			println("Card Type: Tavel Happy");
			return;
		}

		if (ID1 == 0x8F && ID2 == 0x03 && ID3 == 0x68) {
			bCardType = NXPTAG_NTAG213;
			println("Card Type: NXPTAG NTAG213");
			return;
		}

		if (ID1 == 0x8F && ID2 == 0x01 && ID3 == 0x6A) {
			bCardType = MiFare_Classic_1K;
			println("Card Type: MiFare Classic 1K");
			return;
		}

		if (ID1 == 0x81 && ID2 == 0x00 && ID3 == 0x00) {
			bCardType = MickeyBand;
			println("Card Type: Magic Band");
			return;
		}

		println("Card Type Unknown!!!!");
	
	
}
