#include <mcp_can_dfs.h>
#include <mcp_can.h>

#include "src/libcanard/canard.h"
#include "src/libcanard/canard_dsdl.h"

#define NODE_ID 1

// libcanard
CanardInstance canard_instance;
static const uint16_t HeartbeatSubjectID = 7509;
static const uint16_t DummySubjectID = 1914;

// Seeed-Studio/CAN_BUS_Shield
const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

// LED to indicate heartbeat
const int led_pin = PB7;

//Counter and compare values for 1Hz timer interrupt
const uint16_t t4_comp = 15625;
volatile boolean do_publish_heartbeat = false;

// Memory management for libcanard
static void *canardAllocate(CanardInstance *const ins, const size_t amount)
{
    (void)ins;
    return malloc(amount);
}

static void canardFree(CanardInstance *const ins, void *const pointer)
{
    (void)ins;
    free(pointer);
}

void setup()
{

    setupOneHertzTimer();

    // Initialize the node with a static node-ID as specified in the command-line arguments.
    canard_instance = canardInit(&canardAllocate, &canardFree);
    canard_instance.mtu_bytes = CANARD_MTU_CAN_CLASSIC; // Do not use CAN FD to enhance compatibility.
    canard_instance.node_id = (CanardNodeID)NODE_ID;

    //Subsciptions

    // Subscribe to messages uavcan.node.Heartbeat.
    CanardRxSubscription heartbeat_subscription;
    (void)canardRxSubscribe(&canard_instance, //
                            CanardTransferKindMessage,
                            HeartbeatSubjectID,
                            7,
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &heartbeat_subscription);

    // Subcribe to dummy message
    CanardRxSubscription dummy_subscription;
    (void)canardRxSubscribe(&canard_instance,
                            CanardTransferKindMessage,
                            DummySubjectID,
                            0,
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &dummy_subscription);

    Serial.begin(115200);

    // Initialize the CAN bus module
    while (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    Serial.flush();
}

// the loop function runs over and over again forever
void loop()
{
    if (do_publish_heartbeat)
    {
        publishHeartbeat(0);
        do_publish_heartbeat = false;
    }

    // Transmit pending frames.
    const CanardFrame *txf = canardTxPeek(&canard_instance);
    while (txf != NULL)
    {
        canTransmit(txf);
        canardTxPop(&canard_instance);
        free((void *)txf);
        txf = canardTxPeek(&canard_instance);
    }

    // Process received frames, if any.
    CanardFrame rxf;
    if (canRecieve(&rxf))
    {

        Serial.println("*****************");
        CanardTransfer transfer;
        if (canardRxAccept(&canard_instance, &rxf, 0, &transfer))
        {
            if ((transfer.transfer_kind == CanardTransferKindMessage) &&
                (transfer.port_id == HeartbeatSubjectID))
            {
                Serial.print("Hearbeat Recieved\n");
            }
            free((void *)transfer.payload);
        }
        Serial.println("*****************");
    }
}

ISR(TIMER4_COMPA_vect)
{
    PORTB ^= (1 << led_pin);
    do_publish_heartbeat = true;
}

void setupOneHertzTimer()
{
    cli();
    DDRB |= (1 << led_pin);

    // Reset Timer4 Control Reg A and B
    TCCR4A = 0;
    TCCR4B = 0;

    // Set CTC mode
    TCCR4B &= ~(1 << WGM43);
    TCCR4B |= (1 << WGM42);

    // Set to prescaler of 1024
    TCCR4B |= (1 << CS42);
    TCCR4B &= -(1 << CS41);
    TCCR4B |= (1 << CS40);

    // Reset Timer4 and set compare values
    TCNT4 = 0;
    OCR4A = t4_comp;

    // Enalble Timer4 and set compare value
    TIMSK4 = (1 << OCIE4A);
    sei();
}

static void publishHeartbeat(const uint32_t uptime)
{
    static CanardTransferID transfer_id;
    const uint8_t payload[7] = {
        (uint8_t)(uptime >> 0U),
        (uint8_t)(uptime >> 8U),
        (uint8_t)(uptime >> 16U),
        (uint8_t)(uptime >> 24U),
        0,
        0,
        0,
    };
    const CanardTransfer transfer = {
        timestamp_usec : uptime,
        priority : CanardPriorityNominal,
        transfer_kind : CanardTransferKindMessage,
        port_id : HeartbeatSubjectID,
        remote_node_id : CANARD_NODE_ID_UNSET,
        transfer_id : transfer_id,
        payload_size : sizeof(payload),
        payload : &payload[0]
    };
    ++transfer_id;
    canardTxPush(&canard_instance, &transfer);
    //maybe return result of canardTxPush?
}

int canTransmit(const CanardFrame *frame)
{
    byte cansSendStatus = CAN.sendMsgBuf(frame->extended_can_id, 1, frame->payload_size, frame->payload);
    return cansSendStatus;
}

int canRecieve(CanardFrame *out_frame)
{
    int receiver_state = CAN.checkReceive();

    if (CAN_NOMSG == receiver_state)
    {
        return 0;
    }

    if (CAN_MSGAVAIL == receiver_state)
    {
        unsigned char len = 0;
        static unsigned char buf[64];

        // check if data coming
        CAN.readMsgBuf(&len, buf);

        out_frame->extended_can_id = CAN.getCanId();
        out_frame->payload_size = len;
        out_frame->payload = buf;

        unsigned long canId = CAN.getCanId();

        Serial.println("-----------------------------");
        Serial.print("Get data from ID: 0x");
        Serial.println(canId, HEX);

        for (int i = 0; i < len; i++)
        { // print the data
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();
        Serial.flush();

        return 1;
    }

    return -1;
}