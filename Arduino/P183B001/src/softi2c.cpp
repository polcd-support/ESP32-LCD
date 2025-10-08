#include "../inc/softi2C.h"

#define IIC_DELAY 5  // I2C延迟时间(微秒)

// 初始化I2C总线
void IICInit(iic_bus_t *bus)
{
    pinMode(bus->sda_pin, OUTPUT);
    digitalWrite(bus->sda_pin, HIGH);
    
    pinMode(bus->scl_pin, OUTPUT);
    digitalWrite(bus->scl_pin, HIGH);
}

// 发送I2C起始信号
void IICStart(iic_bus_t *bus)
{
    digitalWrite(bus->sda_pin, HIGH);
    digitalWrite(bus->scl_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->sda_pin, LOW);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, LOW);
}

// 发送I2C停止信号
void IICStop(iic_bus_t *bus)
{
    digitalWrite(bus->scl_pin, LOW);
    digitalWrite(bus->sda_pin, LOW);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->sda_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
}

// 等待I2C应答
uint8_t IICWaitAck(iic_bus_t *bus)
{
    uint16_t ucErrTime = 0XFFF;
    
    digitalWrite(bus->scl_pin, LOW);
    pinMode(bus->sda_pin, INPUT_PULLUP);  // 切换为输入模式，启用内部上拉
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, HIGH);

    while (digitalRead(bus->sda_pin))
    {
        if ((ucErrTime--) == 0)
        {
            digitalWrite(bus->sda_pin, HIGH);
            pinMode(bus->sda_pin, OUTPUT);
            IICStop(bus);
            return 1;  // 超时，未收到应答
        }
    }
    
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, LOW);
    digitalWrite(bus->sda_pin, HIGH);
    pinMode(bus->sda_pin, OUTPUT);
    return 0;  // 收到应答
}

// 发送I2C应答信号
void IICAck(iic_bus_t *bus)
{
    digitalWrite(bus->scl_pin, LOW);
    pinMode(bus->sda_pin, OUTPUT);
    digitalWrite(bus->sda_pin, LOW);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, LOW);
}

// 发送I2C非应答信号
void IICNAck(iic_bus_t *bus)
{
    digitalWrite(bus->scl_pin, LOW);
    pinMode(bus->sda_pin, OUTPUT);
    digitalWrite(bus->sda_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, HIGH);
    delayMicroseconds(IIC_DELAY);
    digitalWrite(bus->scl_pin, LOW);
}

// 发送一个字节
void IICSendByte(iic_bus_t *bus, uint8_t txd)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        digitalWrite(bus->scl_pin, LOW);
        delayMicroseconds(IIC_DELAY);
        
        if (txd & 0x80)
            digitalWrite(bus->sda_pin, HIGH);
        else
            digitalWrite(bus->sda_pin, LOW);
            
        digitalWrite(bus->scl_pin, HIGH);
        delayMicroseconds(IIC_DELAY);
        txd <<= 1;
    }
}

// 接收一个字节
uint8_t IICRecvByte(iic_bus_t *bus)
{
    uint8_t i, receive = 0;
    pinMode(bus->sda_pin, INPUT_PULLUP);  // 切换为输入模式，启用内部上拉

    for (i = 0; i < 8; i++)
    {
        receive <<= 1;
        digitalWrite(bus->scl_pin, LOW);
        delayMicroseconds(IIC_DELAY);
        digitalWrite(bus->scl_pin, HIGH);

        if (digitalRead(bus->sda_pin))
            receive |= 0x01;

        delayMicroseconds(IIC_DELAY);
    }

    return receive;
}

// 读取一个字节数据
uint8_t IIC_Read_One_Byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t res;

    IICStart(bus);
    IICSendByte(bus, (dev_addr << 1) & 0xFE);  // 写操作
    IICWaitAck(bus);
    IICSendByte(bus, reg_addr);
    IICWaitAck(bus);

    IICStart(bus);
    IICSendByte(bus, (dev_addr << 1) | 0x01);  // 读操作
    IICWaitAck(bus);
    res = IICRecvByte(bus);
    IICNAck(bus);
    IICStop(bus);

    return res;
}

// 写入一个字节数据
void IIC_Write_One_Byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t dat)
{
    IICStart(bus);
    IICSendByte(bus, (dev_addr << 1) & 0xFE);  // 写操作
    IICWaitAck(bus);
    IICSendByte(bus, reg_addr);
    IICWaitAck(bus);
    IICSendByte(bus, dat);
    IICWaitAck(bus);
    IICStop(bus);
}

// 读取多个字节数据
void IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *buf)
{
    IICStart(bus);
    IICSendByte(bus, (dev_addr << 1) & 0xFE);  // 写操作
    IICWaitAck(bus);
    IICSendByte(bus, reg_addr);
    IICWaitAck(bus);

    IICStart(bus);
    IICSendByte(bus, (dev_addr << 1) | 0x01);  // 读操作
    IICWaitAck(bus);

    while (len)
    {
        if (len == 1)
        {
            *buf = IICRecvByte(bus);
            IICNAck(bus);  // 最后一个字节发送非应答
        }
        else
        {
            *buf = IICRecvByte(bus);
            IICAck(bus);   // 非最后一个字节发送应答
        }
        buf++;
        len--;
    }

    IICStop(bus);
}
