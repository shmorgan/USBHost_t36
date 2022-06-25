#ifndef USB_HOST_CCID_TEENSY36_
#define USB_HOST_CCID_TEENSY36_

//--------------------------------------------------------------------------

#define NO_CARD 				0	
#define CARD_INSERTED 			1
#define SND_POWER_OFF			2
#define RPLY_POWER_OFF			3
#define SND_POWER_ON			4
#define RPLY_POWER_ON			5
#define SND_SET_PARAMETERS		6
#define RPLY_SET_PARAMETERS 	7
#define SND_XFRBLOCK			8
#define RPLY_XFRBLOCK			9
#define CARD_ID_VALID           20

#define CARD_REMOVED			99

#define CARD_IN_SLOT            0x03    

// Card Types Tested So Far
#define NXPTAG_NTAG213          10
#define MiFare_Classic_1K       20
#define MickeyBand              30
#define HOTEL1                  40 

class USBCCIDBase: public USBDriver, public Stream {
	public:

	// FIXME: need different USBCCID, with bigger buffers for 480 Mbit & faster speed
	enum { BUFFER_SIZE = 1648 }; // must hold at least 6 max size packets, plus 2 extra bytes
	enum { DEFAULT_WRITE_TIMEOUT = 3500};
	
	USBCCIDBase(USBHost &host, uint32_t *big_buffer, uint16_t buffer_size, 
		uint16_t min_pipe_rxtx1, uint16_t max_pipe_rxtx1, uint16_t min_pipe_rxtx2, uint16_t max_pipe_rxtx2) :
			txtimer(this), 
			_bigBuffer(big_buffer), 
			_big_buffer_size(buffer_size), 
			_min_rxtx1(min_pipe_rxtx1), 
			_max_rxtx1(max_pipe_rxtx1), 
			_min_rxtx2(min_pipe_rxtx2), 
			_max_rxtx2(max_pipe_rxtx2) 
		{ 

			init(); 
		}

	void begin(void);
	void end(void);
	uint32_t writeTimeout() {return write_timeout_;}
	void writeTimeOut(uint32_t write_timeout) {write_timeout_ = write_timeout;} // Will not impact current ones.
	virtual int available(void);
	virtual int peek(void);
	virtual int read(void);
	virtual int availableForWrite();
	virtual size_t write(uint8_t c);
	virtual void flush1(void);
	virtual void flush2(void);
	virtual uint8_t GetSlotState(void);
	virtual uint8_t GetCardState(void);
	
	virtual void GetCardNumber(uint8_t *buff,uint8_t buffsize);
	uint8_t GetCardNumberLength();
	
	using Print::write;

protected:
	virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
	virtual void control(const Transfer_t *transfer);
	virtual void disconnect();
	virtual void timer_event(USBDriverTimer *whichTimer);
private:
	static void rx_callback1(const Transfer_t *transfer);
	static void tx_callback1(const Transfer_t *transfer);
	static void intr_callback1(const Transfer_t *transfer);
	static void rx_callback2(const Transfer_t *transfer);
	static void tx_callback2(const Transfer_t *transfer);
	static void intr_callback2(const Transfer_t *transfer);

		// Section 6.1 Command Pipe, Bulk-OUT Messages
	void PC_to_RDR_GetSlotStatus(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence);
	void PC_to_RDR_IccPowerOn(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence);
	void PC_to_RDR_IccPowerOff(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence);
	void PC_to_RDR_XferBlock(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence);
	void PC_to_RDR_SetParameters(Pipe_t *Bulk_Out_Pipe, uint8_t slot, uint8_t sequence);
	void PrintTagType(uint8_t ID1, uint8_t ID2, uint8_t ID3);
	
	void rx_data1(const Transfer_t *transfer);
	void tx_data1(const Transfer_t *transfer);
	void intr_data1(const Transfer_t *transfer);

	void rx_data2(const Transfer_t *transfer);
	void tx_data2(const Transfer_t *transfer);
	void intr_data2(const Transfer_t *transfer);

	void rx_queue_packets1(uint32_t head, uint32_t tail);
	void rx_queue_packets2(uint32_t head, uint32_t tail);

	void init();
	static bool check_rxtx_ep1(uint32_t &rxep, uint32_t &txep);
	static bool check_rxtx_ep2(uint32_t &rxep, uint32_t &txep);
	
	bool init_buffers1(uint32_t rsize, uint32_t tsize);
	bool init_buffers2(uint32_t rsize, uint32_t tsize);

private:
	Pipe_t mypipes[8] __attribute__ ((aligned(32)));

	Transfer_t mytransfers[14] __attribute__ ((aligned(32)));
	strbuf_t mystring_bufs[1];

	USBDriverTimer txtimer;

	uint32_t *_bigBuffer;
	uint16_t _big_buffer_size;

	uint16_t _min_rxtx1;
	uint16_t _max_rxtx1;
	uint16_t _min_rxtx2;
	uint16_t _max_rxtx2;

	setup_t setup;
	uint8_t setupdata[16]; // 
	uint32_t baudrate;
	uint32_t write_timeout_ = DEFAULT_WRITE_TIMEOUT;

	Pipe_t *rxpipe1;
	Pipe_t *rxpipe2;
	Pipe_t *txpipe1;
	Pipe_t *txpipe2;
	Pipe_t *intrpipe1;
	Pipe_t *intrpipe2;
	
	uint8_t *rx1_1;	// location for first incoming packet
	uint8_t *rx2_1;	// location for second incoming packet
	uint8_t *rxbuf_1;	// receive circular buffer
	
	uint8_t *tx1_1;	// location for first outgoing packet
	uint8_t *tx2_1;	// location for second outgoing packet
	uint8_t *txbuf_1;
	
	uint8_t *intr1_1;	// location for first outgoing packet
	uint8_t *intr2_1;	// location for second outgoing packet
	uint8_t *intrbuf1_1;

	uint8_t *rx1_2;	// location for first incoming packet
	uint8_t *rx2_2;	// location for second incoming packet
	uint8_t *rxbuf_2;	// receive circular buffer
	
	uint8_t *tx1_2;	// location for first outgoing packet
	uint8_t *tx2_2;	// location for second outgoing packet
	uint8_t *txbuf_2;
	
	uint8_t *intr1_2;	// location for first outgoing packet
	uint8_t *intr2_2;	// location for second outgoing packet
	uint8_t *intrbuf1_2;

	uint8_t intrdatabuff1[8];
	uint8_t intrdatabuff2[8];
	
	uint8_t slot_state;
	
	uint8_t bSeq1;	// Sequence number for Commands
	uint8_t bSeq2;	// Sequence number for Commands
	
	uint8_t bCardState;
	uint8_t bCardType;
	uint8_t bCardLength;
	
	uint8_t bCardNumberValid = 0;
	
	uint8_t bCardNumber[16];
	
	volatile uint16_t rxhead1;// receive head
	volatile uint16_t rxtail1;// receive tail
	volatile uint16_t txhead1;
	volatile uint16_t txtail1;
	volatile uint16_t intrhead1;
	volatile uint16_t intrtail1;

	volatile uint16_t rxhead2;// receive head
	volatile uint16_t rxtail2;// receive tail
	volatile uint16_t txhead2;
	volatile uint16_t txtail2;
	volatile uint16_t intrhead2;
	volatile uint16_t intrtail2;

	uint16_t rxsize1;// size of receive circular buffer
	uint16_t txsize1;// size of transmit circular buffer
	uint16_t intrsize1;// size of transmit circular buffer

	uint16_t rxsize2;// size of receive circular buffer
	uint16_t txsize2;// size of transmit circular buffer
	uint16_t inr1size2;// size of transmit circular buffer

	volatile uint8_t  rxstate1;// bitmask: which receive packets are queued
	volatile uint8_t  txstate1;
	volatile uint8_t  intrstate1;

	volatile uint8_t  rxstate2;// bitmask: which receive packets are queued
	volatile uint8_t  txstate2;
	volatile uint8_t  intrstate2;

	uint8_t pending_control;
	uint8_t setup_state;	// PL2303 - has several steps... Could use pending control?
	uint8_t interface;
	volatile bool 	control_queued;	// Is there already a queued control messaged
	typedef enum { UNKNOWN=0, CIR315A } ccidtype_t;

	ccidtype_t ccidtype;
	

	typedef struct {
		uint16_t 	idVendor;
		uint16_t 	idProduct;
		ccidtype_t 	ccidtype;
		int			claim_at_type;
	} product_vendor_mapping_t;
	static product_vendor_mapping_t pid_vid_mapping[];

};

class USBCCID : public USBCCIDBase {
public:
	USBCCID(USBHost &host) :
		// hard code the normal one to 1 and 64 bytes for most likely most are 64
		USBCCIDBase(host, bigbuffer, sizeof(bigbuffer), 1, 64, 1, 64) {};
private:
	enum { BUFFER_SIZE = 1648 }; // must hold at least 6 max size packets, plus 2 extra bytes
	uint32_t bigbuffer[(BUFFER_SIZE+3)/4];
};

class USBCCID_BigBuffer: public USBCCIDBase {
public:
	// Default to larger than can be handled by other serial, but can overide
	USBCCID_BigBuffer(USBHost &host, uint16_t min_rxtx1=65, uint16_t min_rxtx2=65) :
		// hard code the normal one to 1 and 64 bytes for most likely most are 64
		USBCCIDBase(host, bigbuffer, sizeof(bigbuffer), min_rxtx1, 512, min_rxtx2, 512) {};
private:
	enum { BUFFER_SIZE = 8096 }; // must hold at least 6 max size packets, plus 2 extra bytes
	uint32_t bigbuffer[(BUFFER_SIZE+3)/4];
};

//--------------------------------------------------------------------------

#endif
