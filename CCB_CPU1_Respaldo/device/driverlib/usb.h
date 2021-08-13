//###########################################################################
//
// FILE:     usb.h
//
// TITLE:  Prototypes for the USB Interface Driver.
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.12.00.00 $
// $Release Date: Fri Feb 12 19:03:23 IST 2021 $
// $Copyright:
// Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################
#ifndef USB_H
#define USB_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup usb_api USB
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//  The following are defines for the g_usUSBFlags variable
//
//*****************************************************************************
#define USB_VBUS_VALID          0x0001
#define USB_ID_HOST             0x0002
#define USB_ID_DEVICE           0x0000
#define USB_PFLT_ACTIVE         0x0004

//*****************************************************************************
//
// The following are values that can be passed to USBIntEnableControl() and
// USBIntDisableControl() as the ui32Flags parameter, and are returned from
// USBIntStatusControl().
//
//*****************************************************************************
#define USB_INTCTRL_ALL         0x000003FF  // All control interrupt sources
#define USB_INTCTRL_STATUS      0x000000FF  // Status Interrupts
#define USB_INTCTRL_VBUS_ERR    0x00000080  // VBUS Error
#define USB_INTCTRL_SESSION     0x00000040  // Session Start Detected
#define USB_INTCTRL_SESSION_END 0x00000040  // Session End Detected
#define USB_INTCTRL_DISCONNECT  0x00000020  // Disconnect Detected
#define USB_INTCTRL_CONNECT     0x00000010  // Device Connect Detected
#define USB_INTCTRL_SOF         0x00000008  // Start of Frame Detected
#define USB_INTCTRL_BABBLE      0x00000004  // Babble signaled
#define USB_INTCTRL_RESET       0x00000004  // Reset signaled
#define USB_INTCTRL_RESUME      0x00000002  // Resume detected
#define USB_INTCTRL_SUSPEND     0x00000001  // Suspend detected
#define USB_INTCTRL_MODE_DETECT 0x00000200  // Mode value valid
#define USB_INTCTRL_POWER_FAULT 0x00000100  // Power Fault detected

//*****************************************************************************
//
// The following are values that can be passed to USBIntEnableEndpoint() and
// USBIntDisableEndpoint() as the ui32Flags parameter, and are returned from
// USBIntStatusEndpoint().
//
//*****************************************************************************
#define USB_INTEP_ALL           0xFFFFFFFF  // Host IN Interrupts
#define USB_INTEP_HOST_IN       0xFFFE0000  // Host IN Interrupts
#define USB_INTEP_HOST_IN_15    0x80000000  // Endpoint 15 Host IN Interrupt
#define USB_INTEP_HOST_IN_14    0x40000000  // Endpoint 14 Host IN Interrupt
#define USB_INTEP_HOST_IN_13    0x20000000  // Endpoint 13 Host IN Interrupt
#define USB_INTEP_HOST_IN_12    0x10000000  // Endpoint 12 Host IN Interrupt
#define USB_INTEP_HOST_IN_11    0x08000000  // Endpoint 11 Host IN Interrupt
#define USB_INTEP_HOST_IN_10    0x04000000  // Endpoint 10 Host IN Interrupt
#define USB_INTEP_HOST_IN_9     0x02000000  // Endpoint 9 Host IN Interrupt
#define USB_INTEP_HOST_IN_8     0x01000000  // Endpoint 8 Host IN Interrupt
#define USB_INTEP_HOST_IN_7     0x00800000  // Endpoint 7 Host IN Interrupt
#define USB_INTEP_HOST_IN_6     0x00400000  // Endpoint 6 Host IN Interrupt
#define USB_INTEP_HOST_IN_5     0x00200000  // Endpoint 5 Host IN Interrupt
#define USB_INTEP_HOST_IN_4     0x00100000  // Endpoint 4 Host IN Interrupt
#define USB_INTEP_HOST_IN_3     0x00080000  // Endpoint 3 Host IN Interrupt
#define USB_INTEP_HOST_IN_2     0x00040000  // Endpoint 2 Host IN Interrupt
#define USB_INTEP_HOST_IN_1     0x00020000  // Endpoint 1 Host IN Interrupt

#define USB_INTEP_DEV_OUT       0xFFFE0000  // Device OUT Interrupts
#define USB_INTEP_DEV_OUT_15    0x80000000  // Endpoint 15 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_14    0x40000000  // Endpoint 14 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_13    0x20000000  // Endpoint 13 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_12    0x10000000  // Endpoint 12 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_11    0x08000000  // Endpoint 11 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_10    0x04000000  // Endpoint 10 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_9     0x02000000  // Endpoint 9 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_8     0x01000000  // Endpoint 8 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_7     0x00800000  // Endpoint 7 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_6     0x00400000  // Endpoint 6 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_5     0x00200000  // Endpoint 5 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_4     0x00100000  // Endpoint 4 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_3     0x00080000  // Endpoint 3 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_2     0x00040000  // Endpoint 2 Device OUT Interrupt
#define USB_INTEP_DEV_OUT_1     0x00020000  // Endpoint 1 Device OUT Interrupt

#define USB_INTEP_HOST_OUT      0x0000FFFE  // Host OUT Interrupts
#define USB_INTEP_HOST_OUT_15   0x00008000  // Endpoint 15 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_14   0x00004000  // Endpoint 14 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_13   0x00002000  // Endpoint 13 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_12   0x00001000  // Endpoint 12 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_11   0x00000800  // Endpoint 11 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_10   0x00000400  // Endpoint 10 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_9    0x00000200  // Endpoint 9 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_8    0x00000100  // Endpoint 8 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_7    0x00000080  // Endpoint 7 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_6    0x00000040  // Endpoint 6 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_5    0x00000020  // Endpoint 5 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_4    0x00000010  // Endpoint 4 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_3    0x00000008  // Endpoint 3 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_2    0x00000004  // Endpoint 2 Host OUT Interrupt
#define USB_INTEP_HOST_OUT_1    0x00000002  // Endpoint 1 Host OUT Interrupt

#define USB_INTEP_DEV_IN        0x0000FFFE  // Device IN Interrupts
#define USB_INTEP_DEV_IN_15     0x00008000  // Endpoint 15 Device IN Interrupt
#define USB_INTEP_DEV_IN_14     0x00004000  // Endpoint 14 Device IN Interrupt
#define USB_INTEP_DEV_IN_13     0x00002000  // Endpoint 13 Device IN Interrupt
#define USB_INTEP_DEV_IN_12     0x00001000  // Endpoint 12 Device IN Interrupt
#define USB_INTEP_DEV_IN_11     0x00000800  // Endpoint 11 Device IN Interrupt
#define USB_INTEP_DEV_IN_10     0x00000400  // Endpoint 10 Device IN Interrupt
#define USB_INTEP_DEV_IN_9      0x00000200  // Endpoint 9 Device IN Interrupt
#define USB_INTEP_DEV_IN_8      0x00000100  // Endpoint 8 Device IN Interrupt
#define USB_INTEP_DEV_IN_7      0x00000080  // Endpoint 7 Device IN Interrupt
#define USB_INTEP_DEV_IN_6      0x00000040  // Endpoint 6 Device IN Interrupt
#define USB_INTEP_DEV_IN_5      0x00000020  // Endpoint 5 Device IN Interrupt
#define USB_INTEP_DEV_IN_4      0x00000010  // Endpoint 4 Device IN Interrupt
#define USB_INTEP_DEV_IN_3      0x00000008  // Endpoint 3 Device IN Interrupt
#define USB_INTEP_DEV_IN_2      0x00000004  // Endpoint 2 Device IN Interrupt
#define USB_INTEP_DEV_IN_1      0x00000002  // Endpoint 1 Device IN Interrupt

#define USB_INTEP_0             0x00000001  // Endpoint 0 Interrupt

//*****************************************************************************
//
// The following are values that are returned from USBSpeedGet().
//
//*****************************************************************************
#define USB_UNDEF_SPEED         0x80000000  // Current speed is undefined
#define USB_FULL_SPEED          0x00000001  // Current speed is Full Speed
#define USB_LOW_SPEED           0x00000000  // Current speed is Low Speed

//*****************************************************************************
//
// The following are values that are returned from USBEndpointStatus().  The
// USB_HOST_* values are used when the USB controller is in host mode and the
// USB_DEV_* values are used when the USB controller is in device mode.
//
//*****************************************************************************
#define USB_HOST_IN_STATUS      0xFFFF0000  // Mask of all host IN interrupts
#define USB_HOST_IN_PID_ERROR   0x10000000  // Stall on this endpoint received
#define USB_HOST_IN_NOT_COMP    0x01000000  // Device failed to respond
#define USB_HOST_IN_STALL       0x00400000  // Stall on this endpoint received
#define USB_HOST_IN_DATA_ERROR  0x00080000  // CRC or bit-stuff error
                                            // (ISOC Mode)
#define USB_HOST_IN_NAK_TO      0x00080000  // NAK received for more than the
                                            // specified timeout period
#define USB_HOST_IN_ERROR       0x00040000  // Failed to communicate with a
                                            // device
#define USB_HOST_IN_FIFO_FULL   0x00020000  // RX FIFO full
#define USB_HOST_IN_PKTRDY      0x00010000  // Data packet ready
#define USB_HOST_OUT_STATUS     0x0000FFFF  // Mask of all host OUT interrupts
#define USB_HOST_OUT_NAK_TO     0x00000080  // NAK received for more than the
                                            // specified timeout period
#define USB_HOST_OUT_NOT_COMP   0x00000080  // No response from device
                                            // (ISOC mode)
#define USB_HOST_OUT_STALL      0x00000020  // Stall on this endpoint received
#define USB_HOST_OUT_ERROR      0x00000004  // Failed to communicate with a
                                            // device
#define USB_HOST_OUT_FIFO_NE    0x00000002  // TX FIFO is not empty
#define USB_HOST_OUT_PKTPEND    0x00000001  // Transmit still being transmitted
#define USB_HOST_EP0_NAK_TO     0x00000080  // NAK received for more than the
                                            // specified timeout period
#define USB_HOST_EP0_STATUS     0x00000040  // This was a status packet
#define USB_HOST_EP0_ERROR      0x00000010  // Failed to communicate with a
                                            // device
#define USB_HOST_EP0_RX_STALL   0x00000004  // Stall on this endpoint received
#define USB_HOST_EP0_RXPKTRDY   0x00000001  // Receive data packet ready
#define USB_DEV_RX_PID_ERROR    0x01000000  // PID error in isochronous
                                            // transfer
#define USB_DEV_RX_SENT_STALL   0x00400000  // Stall was sent on this endpoint
#define USB_DEV_RX_DATA_ERROR   0x00080000  // CRC error on the data
#define USB_DEV_RX_OVERRUN      0x00040000  // OUT packet was not loaded due to
                                            // a full FIFO
#define USB_DEV_RX_FIFO_FULL    0x00020000  // RX FIFO full
#define USB_DEV_RX_PKT_RDY      0x00010000  // Data packet ready
#define USB_DEV_TX_NOT_COMP     0x00000080  // Large packet split up, more data
                                            // to come
#define USB_DEV_TX_SENT_STALL   0x00000020  // Stall was sent on this endpoint
#define USB_DEV_TX_UNDERRUN     0x00000004  // IN received with no data ready
#define USB_DEV_TX_FIFO_NE      0x00000002  // The TX FIFO is not empty
#define USB_DEV_TX_TXPKTRDY     0x00000001  // Transmit still being transmitted
#define USB_DEV_EP0_SETUP_END   0x00000010  // Control transaction ended before
                                            // Data End seen
#define USB_DEV_EP0_SENT_STALL  0x00000004  // Stall was sent on this endpoint
#define USB_DEV_EP0_IN_PKTPEND  0x00000002  // Transmit data packet pending
#define USB_DEV_EP0_OUT_PKTRDY  0x00000001  // Receive data packet ready

//*****************************************************************************
//
// The following are values that can be passed to USBHostEndpointConfig() and
// USBDevEndpointConfigSet() as the ui32Flags parameter.
//
//*****************************************************************************
#define USB_EP_AUTO_SET         0x00000001  // Auto set feature enabled
#define USB_EP_AUTO_REQUEST     0x00000002  // Auto request feature enabled
#define USB_EP_AUTO_CLEAR       0x00000004  // Auto clear feature enabled
#define USB_EP_DMA_MODE_0       0x00000008  // Enable DMA access using mode 0
#define USB_EP_DMA_MODE_1       0x00000010  // Enable DMA access using mode 1
#define USB_EP_MODE_ISOC        0x00000000  // Isochronous endpoint
#define USB_EP_MODE_BULK        0x00000100  // Bulk endpoint
#define USB_EP_MODE_INT         0x00000200  // Interrupt endpoint
#define USB_EP_MODE_CTRL        0x00000300  // Control endpoint
#define USB_EP_MODE_MASK        0x00000300  // Mode Mask
#define USB_EP_SPEED_LOW        0x00000000  // Low Speed
#define USB_EP_SPEED_FULL       0x00001000  // Full Speed
#define USB_EP_HOST_IN          0x00000000  // Host IN endpoint
#define USB_EP_HOST_OUT         0x00002000  // Host OUT endpoint
#define USB_EP_DEV_IN           0x00002000  // Device IN endpoint
#define USB_EP_DEV_OUT          0x00000000  // Device OUT endpoint

//*****************************************************************************
//
// The following are values that can be passed to USBHostPwrConfig() as the
// ui32Flags parameter.
//
//*****************************************************************************
#define USB_HOST_PWRFLT_LOW     0x00000010
#define USB_HOST_PWRFLT_HIGH    0x00000030
#define USB_HOST_PWRFLT_EP_NONE 0x00000000
#define USB_HOST_PWRFLT_EP_TRI  0x00000140
#define USB_HOST_PWRFLT_EP_LOW  0x00000240
#define USB_HOST_PWRFLT_EP_HIGH 0x00000340
#define USB_HOST_PWREN_MAN_LOW  0x00000000
#define USB_HOST_PWREN_MAN_HIGH 0x00000001
#define USB_HOST_PWREN_AUTOLOW  0x00000002
#define USB_HOST_PWREN_AUTOHIGH 0x00000003
#define USB_HOST_PWREN_FILTER   0x00010000

//*****************************************************************************
//
// This value specifies the maximum size of transfers on endpoint 0 as 64
// bytes.  This value is fixed in hardware as the FIFO size for endpoint 0.
//
//*****************************************************************************
#define MAX_PACKET_SIZE_EP0     64

//*****************************************************************************
//
// These values are used to indicate which endpoint to access.
//
//*****************************************************************************
#define USB_EP_0                0x00000000  // Endpoint 0
#define USB_EP_1                0x00000010  // Endpoint 1
#define USB_EP_2                0x00000020  // Endpoint 2
#define USB_EP_3                0x00000030  // Endpoint 3
#define USB_EP_4                0x00000040  // Endpoint 4
#define USB_EP_5                0x00000050  // Endpoint 5
#define USB_EP_6                0x00000060  // Endpoint 6
#define USB_EP_7                0x00000070  // Endpoint 7
#define USB_EP_8                0x00000080  // Endpoint 8
#define USB_EP_9                0x00000090  // Endpoint 9
#define USB_EP_10               0x000000A0  // Endpoint 10
#define USB_EP_11               0x000000B0  // Endpoint 11
#define USB_EP_12               0x000000C0  // Endpoint 12
#define USB_EP_13               0x000000D0  // Endpoint 13
#define USB_EP_14               0x000000E0  // Endpoint 14
#define USB_EP_15               0x000000F0  // Endpoint 15
#define NUM_USB_EP              16           // Number of supported endpoints

//*****************************************************************************
//
// These macros allow conversion between 0-based endpoint indices and the
// USB_EP_x values required when calling various USB APIs.
//
//*****************************************************************************
#define IndexToUSBEP(x)         (((uint32_t)(x) << 4) & 0xFF)
#define USBEPToIndex(x)         ((x) >> 4)

//*****************************************************************************
//
// The following are values that can be passed to USBFIFOConfigSet() as the
// ui32FIFOSize parameter.
//
//*****************************************************************************
#define USB_FIFO_SZ_8           0x00000000  // 8 byte FIFO
#define USB_FIFO_SZ_16          0x00000001  // 16 byte FIFO
#define USB_FIFO_SZ_32          0x00000002  // 32 byte FIFO
#define USB_FIFO_SZ_64          0x00000003  // 64 byte FIFO
#define USB_FIFO_SZ_128         0x00000004  // 128 byte FIFO
#define USB_FIFO_SZ_256         0x00000005  // 256 byte FIFO
#define USB_FIFO_SZ_512         0x00000006  // 512 byte FIFO
#define USB_FIFO_SZ_1024        0x00000007  // 1024 byte FIFO
#define USB_FIFO_SZ_2048        0x00000008  // 2048 byte FIFO
#define USB_FIFO_SZ_4096        0x00000009  // 4096 byte FIFO
#define USB_FIFO_SZ_8_DB        0x00000010  // 8 byte double buffered FIFO
                                            // (occupying 16 bytes)
#define USB_FIFO_SZ_16_DB       0x00000011  // 16 byte double buffered FIFO
                                            // (occupying 32 bytes)
#define USB_FIFO_SZ_32_DB       0x00000012  // 32 byte double buffered FIFO
                                            // (occupying 64 bytes)
#define USB_FIFO_SZ_64_DB       0x00000013  // 64 byte double buffered FIFO
                                            // (occupying 128 bytes)
#define USB_FIFO_SZ_128_DB      0x00000014  // 128 byte double buffered FIFO
                                            // (occupying 256 bytes)
#define USB_FIFO_SZ_256_DB      0x00000015  // 256 byte double buffered FIFO
                                            // (occupying 512 bytes)
#define USB_FIFO_SZ_512_DB      0x00000016  // 512 byte double buffered FIFO
                                            // (occupying 1024 bytes)
#define USB_FIFO_SZ_1024_DB     0x00000017  // 1024 byte double buffered FIFO
                                            // (occupying 2048 bytes)
#define USB_FIFO_SZ_2048_DB     0x00000018  // 2048 byte double buffered FIFO
                                            // (occupying 4096 bytes)

//*****************************************************************************
//
// This macro allow conversion from a FIFO size label as defined above to
// a number of bytes
//
//*****************************************************************************
#define USB_FIFO_SIZE_DB_FLAG  0x00000010
#define USBFIFOSizeToBytes(x)   ((uint32_t)8 << (x))

//*****************************************************************************
//
// The following are values that can be passed to USBEndpointDataSend() as the
// ui32TransType parameter.
//
//*****************************************************************************
#define USB_TRANS_OUT           0x00000102  // Normal OUT transaction
#define USB_TRANS_IN            0x00000102  // Normal IN transaction
#define USB_TRANS_IN_LAST       0x0000010a  // Final IN transaction (for
                                            // endpoint 0 in device mode)
#define USB_TRANS_SETUP         0x0000110a  // Setup transaction (for endpoint
                                            // 0)
#define USB_TRANS_STATUS        0x00000142  // Status transaction (for endpoint
                                            // 0)

//*****************************************************************************
//
// The following are values are returned by the USBModeGet function.
//
//*****************************************************************************
#define USB_DUAL_MODE_HOST      0x00000001  // Dual mode controller is in Host
                                            // mode.
#define USB_DUAL_MODE_DEVICE    0x00000081  // Dual mode controller is in
                                            // Device mode.
#define USB_DUAL_MODE_NONE      0x00000080  // Dual mode controller mode is not
                                            // set.
#define USB_OTG_MODE_ASIDE_HOST 0x0000001d  // OTG controller on the A side of
                                            // the cable.
#define USB_OTG_MODE_ASIDE_NPWR 0x00000001  // OTG controller on the A side of
                                            // the cable.
#define USB_OTG_MODE_ASIDE_SESS 0x00000009  // OTG controller on the A side of
                                            // the cable Session Valid.
#define USB_OTG_MODE_ASIDE_AVAL 0x00000011  // OTG controller on the A side of
                                            // the cable A valid.
#define USB_OTG_MODE_ASIDE_DEV  0x00000019  // OTG controller on the A side of
                                            // the cable.
#define USB_OTG_MODE_BSIDE_HOST 0x0000009d  // OTG controller on the B side of
                                            // the cable.
#define USB_OTG_MODE_BSIDE_DEV  0x00000099  // OTG controller on the B side of
                                            // the cable.
#define USB_OTG_MODE_BSIDE_NPWR 0x00000081  // OTG controller on the B side of
                                            // the cable.
#define USB_OTG_MODE_NONE       0x00000080  // OTG controller mode is not set.


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern uint32_t USBDevAddrGet(uint32_t ui32Base);
extern void USBDevAddrSet(uint32_t ui32Base, uint32_t ui32Address);
extern void USBDevConnect(uint32_t ui32Base);
extern void USBDevDisconnect(uint32_t ui32Base);
extern void USBDevEndpointConfigSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                    uint32_t ui32MaxPacketSize,
                                    uint32_t ui32Flags);
extern void USBDevEndpointConfigGet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                    uint32_t *pui32MaxPacketSize,
                                    uint32_t *pui32Flags);
extern void USBDevEndpointDataAck(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  bool bIsLastPacket);
extern void USBDevEndpointStall(uint32_t ui32Base, uint32_t ui32Endpoint,
                                uint32_t ui32Flags);
extern void USBDevEndpointStallClear(uint32_t ui32Base, uint32_t ui32Endpoint,
                                     uint32_t ui32Flags);
extern void USBDevEndpointStatusClear(uint32_t ui32Base, uint32_t ui32Endpoint,
                                      uint32_t ui32Flags);
extern uint32_t USBEndpointDataAvail(uint32_t ui32Base, uint32_t ui32Endpoint);
extern void USBEndpointDMAEnable(uint32_t ui32Base, uint32_t ui32Endpoint,
                                 uint32_t ui32Flags);
extern void USBEndpointDMADisable(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint32_t ui32Flags);
extern void USBEndpointDMAConfigSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                    uint32_t ui32Config);
extern int32_t USBEndpointDataGet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint8_t *pui8Data, uint32_t *pui32Size);
extern int32_t USBEndpointDataPut(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint8_t *pui8Data, uint32_t ui32Size);
extern int32_t USBEndpointDataSend(uint32_t ui32Base, uint32_t ui32Endpoint,
                                   uint32_t ui32TransType);
extern void USBEndpointDataToggleClear(uint32_t ui32Base,
                                       uint32_t ui32Endpoint,
                                       uint32_t ui32Flags);
extern void USBEndpointPacketCountSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                      uint32_t ui32Count);
extern uint32_t USBEndpointStatus(uint32_t ui32Base, uint32_t ui32Endpoint);
extern uint32_t USBFIFOAddrGet(uint32_t ui32Base, uint32_t ui32Endpoint);
extern void USBFIFOConfigGet(uint32_t ui32Base, uint32_t ui32Endpoint,
                             uint32_t *pui32FIFOAddress,
                             uint32_t *pui32FIFOSize, uint32_t ui32Flags);
extern void USBFIFOConfigSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                             uint32_t ui32FIFOAddress, uint32_t ui32FIFOSize,
                             uint32_t ui32Flags);
extern void USBFIFOFlush(uint32_t ui32Base, uint32_t ui32Endpoint,
                         uint32_t ui32Flags);
extern uint32_t USBFrameNumberGet(uint32_t ui32Base);
extern uint32_t USBHostAddrGet(uint32_t ui32Base, uint32_t ui32Endpoint,
                               uint32_t ui32Flags);
extern void USBHostAddrSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                           uint32_t ui32Addr, uint32_t ui32Flags);
extern void USBHostEndpointConfig(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint32_t ui32MaxPacketSize,
                                  uint32_t ui32NAKPollInterval,
                                  uint32_t ui32TargetEndpoint,
                                  uint32_t ui32Flags);
extern void USBHostEndpointDataAck(uint32_t ui32Base,
                                   uint32_t ui32Endpoint);
extern void USBHostEndpointDataToggle(uint32_t ui32Base, uint32_t ui32Endpoint,
                                      bool bDataToggle, uint32_t ui32Flags);
extern void USBHostEndpointStatusClear(uint32_t ui32Base,
                                       uint32_t ui32Endpoint,
                                       uint32_t ui32Flags);
extern uint32_t USBHostHubAddrGet(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint32_t ui32Flags);
extern void USBHostHubAddrSet(uint32_t ui32Base, uint32_t ui32Endpoint,
                              uint32_t ui32Addr, uint32_t ui32Flags);
extern void USBHostPwrDisable(uint32_t ui32Base);
extern void USBHostPwrEnable(uint32_t ui32Base);
extern void USBHostPwrConfig(uint32_t ui32Base, uint32_t ui32Flags);
extern void USBHostPwrFaultDisable(uint32_t ui32Base);
extern void USBHostPwrFaultEnable(uint32_t ui32Base);
extern void USBHostRequestIN(uint32_t ui32Base, uint32_t ui32Endpoint);
extern void USBHostRequestINClear(uint32_t ui32Base, uint32_t ui32Endpoint);
extern void USBHostRequestStatus(uint32_t ui32Base);
extern void USBHostReset(uint32_t ui32Base, bool bStart);
extern void USBHostResume(uint32_t ui32Base, bool bStart);
extern uint32_t USBHostSpeedGet(uint32_t ui32Base);
extern void USBHostSuspend(uint32_t ui32Base);
extern void USBIntDisableControl(uint32_t ui32Base, uint32_t ui32IntFlags);
extern void USBIntEnableControl(uint32_t ui32Base, uint32_t ui32IntFlags);
extern uint32_t USBIntStatus(uint32_t ui32Base, uint32_t *ui32IntStatusEP);
extern uint32_t USBIntStatusControl(uint32_t ui32Base);
extern void USBIntDisableEndpoint(uint32_t ui32Base, uint32_t ui32IntFlags);
extern void USBIntEnableEndpoint(uint32_t ui32Base, uint32_t ui32IntFlags);
extern uint32_t USBIntStatusEndpoint(uint32_t ui32Base);
extern void USBOTGSessionRequest(uint32_t ui32Base, bool bStart);
extern uint32_t USBModeGet(uint32_t ui32Base);
extern void USBEndpointDMAChannel(uint32_t ui32Base, uint32_t ui32Endpoint,
                                  uint32_t ui32Channel);
extern void USBHostMode(uint32_t ui32Base);
extern void USBDevMode(uint32_t ui32Base);
extern void USBOTGMode(uint32_t ui32Base);
extern void USBPHYPowerOff(uint32_t ui32Base);
extern void USBPHYPowerOn(uint32_t ui32Base);
extern uint32_t USBNumEndpointsGet(uint32_t ui32Base);

//*****************************************************************************
//
// The following are values that can be passed to USBIntEnable() and
// USBIntDisable() as the ulIntFlags parameter, and are returned from
// USBIntStatus().
//
//*****************************************************************************
#define USB_INT_ALL             0xFF030E0F  // All Interrupt sources
#define USB_INT_STATUS          0xFF000000  // Status Interrupts
#define USB_INT_VBUS_ERR        0x80000000  // VBUS Error
#define USB_INT_SESSION_START   0x40000000  // Session Start Detected
#define USB_INT_SESSION_END     0x20000000  // Session End Detected
#define USB_INT_DISCONNECT      0x20000000  // Disconnect Detected
#define USB_INT_CONNECT         0x10000000  // Device Connect Detected
#define USB_INT_SOF             0x08000000  // Start of Frame Detected
#define USB_INT_BABBLE          0x04000000  // Babble signaled
#define USB_INT_RESET           0x04000000  // Reset signaled
#define USB_INT_RESUME          0x02000000  // Resume detected
#define USB_INT_SUSPEND         0x01000000  // Suspend detected
#define USB_INT_MODE_DETECT     0x00020000  // Mode value valid
#define USB_INT_POWER_FAULT     0x00010000  // Power Fault detected
#define USB_INT_HOST_IN         0x00000E00  // Host IN Interrupts
#define USB_INT_DEV_OUT         0x00000E00  // Device OUT Interrupts
#define USB_INT_HOST_IN_EP3     0x00000800  // Endpoint 3 Host IN Interrupt
#define USB_INT_HOST_IN_EP2     0x00000400  // Endpoint 2 Host IN Interrupt
#define USB_INT_HOST_IN_EP1     0x00000200  // Endpoint 1 Host IN Interrupt
#define USB_INT_DEV_OUT_EP3     0x00000800  // Endpoint 3 Device OUT Interrupt
#define USB_INT_DEV_OUT_EP2     0x00000400  // Endpoint 2 Device OUT Interrupt
#define USB_INT_DEV_OUT_EP1     0x00000200  // Endpoint 1 Device OUT Interrupt
#define USB_INT_HOST_OUT        0x0000000E  // Host OUT Interrupts
#define USB_INT_DEV_IN          0x0000000E  // Device IN Interrupts
#define USB_INT_HOST_OUT_EP3    0x00000008  // Endpoint 3 HOST_OUT Interrupt
#define USB_INT_HOST_OUT_EP2    0x00000004  // Endpoint 2 HOST_OUT Interrupt
#define USB_INT_HOST_OUT_EP1    0x00000002  // Endpoint 1 HOST_OUT Interrupt
#define USB_INT_DEV_IN_EP3      0x00000008  // Endpoint 3 DEV_IN Interrupt
#define USB_INT_DEV_IN_EP2      0x00000004  // Endpoint 2 DEV_IN Interrupt
#define USB_INT_DEV_IN_EP1      0x00000002  // Endpoint 1 DEV_IN Interrupt
#define USB_INT_EP0             0x00000001  // Endpoint 0 Interrupt

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // USB_H
