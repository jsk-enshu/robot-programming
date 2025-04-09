/**
*	@file IcsHardSerialClass.cpp
*	@brief ICS3.5/3.6 arduino library use HardwareSerial
*	@author Kondo Kagaku Co.,Ltd.
*	@date	2017/12/27
*	@version 2.0.0
*	@copyright &copy; Kondo Kagaku Co.,Ltd. 2017
**/


#include <Arduino.h>
#include "IcsHardSerialClass.h"

/**
*	@brief コンストラクタ
**/
IcsHardSerialClass::IcsHardSerialClass()
{
	
}




/**
*	@brief コンストラクタ
*	@param[in] *icsSerial ICSに設定するUART(HardwareSerial型のポインタ)
* 	@param[in] enpin 送受信切替えピンのピン番号
**/
IcsHardSerialClass::IcsHardSerialClass(HardwareSerial *icsSerial,byte enpin)
{
  icsHardSerial = icsSerial;
  enPin = enpin;
}

/**
* @brief コンストラクタ
* @param[in] *hardSerial ICSに設定するUART(HardwareSerial型のポインタ)
* @param[in] enpin 送受信切替えピンのピン番号
* @param[in] baudrate サーボの通信速度
* @param[in] timeout 受信タイムアウト(ms)

**/
IcsHardSerialClass::IcsHardSerialClass(HardwareSerial *hardSerial,byte enpin, long baudrate, int timeout)
{
  icsHardSerial = hardSerial;
  enPin = enpin;
  baudRate = baudrate;
  timeOut = timeout;
}


/**
* @brief デストラクタ
* @post 使用していたのicsHardSerialを開放します
**/
IcsHardSerialClass::~IcsHardSerialClass()
{
  if (icsHardSerial)  //定義してあったらSerialを解放する
  {
    icsHardSerial->end();
  }
}



//初期設定 　void setup()へ　///////////////////////////////////////////////////////////////////////////////
/**
* @brief 通信の初期設定
* @retval true 通信設定完了
* @retval false 通信設定失敗
* @attention Serialの設定、通信速度、タイムアウトは事前に設定してある事
**/
bool IcsHardSerialClass::begin()
{
  if (icsHardSerial == nullptr)
  {
    return false;
  }

  icsHardSerial->begin(baudRate,SERIAL_8E1);
  icsHardSerial->setTimeout(timeOut);
  pinMode(enPin, OUTPUT);
  enLow();
  
	
  return true;
}

/**
* @brief 通信の初期設定
* @param[in] baudrate ICSの通信速度(115200,625000,1250000(1.25M)bps)
* @param[in] timeout 受信タイムアウト(ms)
* @retval true 通信設定完了
* @retval false 通信設定失敗
* @attention UARTピンおよび送受信切替えピンは設定してある事
**/
bool IcsHardSerialClass::begin(long baudrate,int timeout)
{
  baudRate =  baudrate;
  timeOut  = timeout;
  return begin();
}

/**
* @brief 通信の初期設定
* @param[in] *serial ICSに設定するUART(HardwareSerial型のポインタ)
* @param[in] enpin 送受信切替えピンのピン番号
* @param[in] baudrate ICSの通信速度(115200,625000,1250000(1.25M)bps)
* @param[in] timeout 受信タイムアウト(ms)
* @retval true 通信設定完了
* @retval false 通信設定失敗
**/
bool IcsHardSerialClass::begin(HardwareSerial *serial,int enpin,long baudrate,int timeout)
{
  icsHardSerial = serial;
  enPin = enpin;
  baudRate =  baudrate;
  timeOut  = timeout;
  return begin();
}



//データ送受信 /////////////////////////////////////////////////////////////////////////////////////////////
/**
* @brief ICS通信の送受信
* @param[in,out] *txBuf
* @param[in] txLen
* @param[out] *rxBuf 受信格納バッファ
* @param[in] rxLen  受信データ数
* @retval true 通信成功
* @retval false 通信失敗
* @attention 送信データ数、受信データ数はコマンドによって違うので注意する
* @date 2020/02/18 protectedからpublicに変更
**/
bool IcsHardSerialClass::synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen)
{
	int rxSize; //受信数

	//シリアル初期化確認
  
	if(icsHardSerial == nullptr )
	{
		return false;
	}

	icsHardSerial->flush(); //待つ
	enHigh(); //送信切替
	icsHardSerial->write(txBuf, txLen);
	icsHardSerial->flush();   //待つ
	
	while (icsHardSerial->available() > 0) //受信バッファを消す
	{
		// buff = icsSerial->read();	//空読み
		icsHardSerial->read();		//空読み
	}

	enLow();  //受信切替


	rxSize = icsHardSerial->readBytes(rxBuf, rxLen);

	if (rxSize != rxLen) //受信数確認
	{
		return false;
	}
	return true;

	
}






