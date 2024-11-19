/**
*	@file IcsSoftSerialClass.cpp
*	@brief ICS3.5/3.6 arduino library use SoftwareSeria 
*	@author Kondo Kagaku Co.,Ltd.
*	@date	2020/02/20
*	@version 2.1.0
*	@copyright Kondo Kagaku Co.,Ltd. 2017
**/


#include <Arduino.h>
#include <KoCustomSoftSerial.h>
#include "IcsSoftSerialClass.h"


/**
*	@brief コンストラクタ
**/
IcsSoftSerialClass::IcsSoftSerialClass()
{
	icsSoftSerial = nullptr;
}

/**
* @brief コンストラクタ
* @param[in] rxPin SoftSerialのRXに割り当てるピン番号
* @param[in] txPin SoftSerialのTXに割り当てるピン番号 
* @param[in] enpin 送受信切替えピンのピン番号
**/
IcsSoftSerialClass::IcsSoftSerialClass(byte rxPin,byte txPin,byte enpin)
{
	icsSoftSerial = (KoCustomSoftSerial *)new KoCustomSoftSerial(rxPin,txPin);
	icsSoftSerial->setTX(txPin);
	icsSoftSerial->setRX(rxPin);
	g_enpin = enpin;
	pinMode(g_enpin, OUTPUT);
	enLow();
}

/**
* @brief コンストラクタ
* @param[in] rxPin SoftSerialのRXに割り当てるピン番号
* @param[in] txPin SoftSerialのTXに割り当てるピン番号
* @param[in] enpin 送受信切替えピンのピン番号
* @param[in] baudrate サーボの通信速度
* @param[in] timeout 受信タイムアウト(ms)
**/
IcsSoftSerialClass::IcsSoftSerialClass(byte rxPin,byte txPin,byte enpin, long baudrate, int timeout)
{
	icsSoftSerial = (KoCustomSoftSerial *)new KoCustomSoftSerial(rxPin,txPin);
	g_baudrate = baudrate;
	g_timeout = timeout;
	g_enpin = enpin;
	pinMode(g_enpin, OUTPUT);
	enLow();
}


/**
*	@brief コンストラクタ
*	@param[in] *softSerial ICSに設定するUART(KoCustomSoftSerial型のポインタ)
* @param[in] enpin 送受信切替えピンのピン番号
**/
IcsSoftSerialClass::IcsSoftSerialClass(KoCustomSoftSerial *softSerial,byte enpin)
{
  icsSoftSerial = softSerial;
  g_enpin = enpin;
  pinMode(g_enpin, OUTPUT);
  enLow();
}

/**
* @brief コンストラクタ
* @param[in] *softSerial ICSに設定するUART(KoCustomSoftSerial型のポインタ)
* @param[in] enpin 送受信切替えピンのピン番号
* @param[in] baudrate サーボの通信速度
* @param[in] timeout 受信タイムアウト(ms)
**/
IcsSoftSerialClass::IcsSoftSerialClass(KoCustomSoftSerial *softSerial,byte enpin, long baudrate, int timeout)
{
	icsSoftSerial = softSerial;
	g_enpin = enpin;
	g_baudrate = baudrate;
	g_timeout = timeout;
	pinMode(g_enpin, OUTPUT);
	enLow();
}


/**
* @brief デストラクタ
* @post 使用していたのicsSoftSerialを開放します
**/
IcsSoftSerialClass::~IcsSoftSerialClass()
{
	
}



//初期設定 　void setup()へ　///////////////////////////////////////////////////////////////////////////////
/**
* @brief 通信の初期設定
* @retval true 通信設定完了
* @retval false 通信設定失敗
* @attention ピンの割り当て、通信速度、タイムアウトは事前に設定してある事
**/
bool IcsSoftSerialClass::begin()
{

  if (icsSoftSerial == nullptr)
  {
  
    return false;
  }

  icsSoftSerial->begin(g_baudrate);
  icsSoftSerial->setTimeout(g_timeout);
  pinMode(g_enpin, OUTPUT);
  enLow();
  
  

	//icsSoftSerial->listen();
	
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
bool IcsSoftSerialClass::begin(long baudrate,int timeout)
{
  g_baudrate =  baudrate;
  g_timeout  = timeout;
  return begin();
}

/**
* @brief 通信の初期設定
* @param[in] *serial ICSに設定するUART(KoCustomSoftSerial型のポインタ)
* @param[in] enpin 送受信切替えピンのピン番号
* @param[in] baudrate ICSの通信速度(115200,625000,1250000(1.25M)bps)
* @param[in] timeout 受信タイムアウト(ms)
* @retval true 通信設定完了
* @retval false 通信設定失敗
**/
bool IcsSoftSerialClass::begin(KoCustomSoftSerial *serial,byte enpin,long baudrate,int timeout)
{
  icsSoftSerial = serial;
  g_enpin = enpin;
  g_baudrate =  baudrate;
  g_timeout  = timeout;
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
**/
bool IcsSoftSerialClass::synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen)
{

  //シリアル初期化確認
  //if (icsSerial == false)return false;



	if(icsSoftSerial == nullptr )
	{
	
		return false;
	}
	
		


	

  
	enHigh(); //送信切替

	for(byte i = 0; i < txLen; i++)
	{
		icsSoftSerial->write(txBuf[i]);
	}

	enLow();  //受信切替
	
	icsSoftSerial->flush(); //待つ
	icsSoftSerial->set_recv_length(rxLen);		//受信数のセット
	icsSoftSerial->listen();			//取得開始

	uint8_t	ptr = 0;
	unsigned long _startMillis = millis();
	do 
	{
	 if((millis() - _startMillis) > (unsigned long)g_timeout)	break;
		ptr = icsSoftSerial->available();
	} while (ptr == 0);		//データがすべてたまるまでループする(available()自体データ数がおかしいときがある)
	
	//Serial.println(ptr,DEC);

	if(!((ptr == 0xFF) || (ptr == 0)))		//パリティエラーか受信失敗
	{
		ptr &= 0x7f;						//そもそもマイナス側になることがない
		
		for (uint8_t	c = 0;	c < ptr	; c++)	//available()自体データ数がおかしいときがあるので受信数でまわす
		{
			rxBuf[c] =icsSoftSerial->read(); 
			
			//Serial.println(rxBuf[c],HEX);
			
		}
		//Serial.write("\r\n");

		return true;				
	}
	else
	{//ﾀｲﾑｱｳﾄ
		//Serial.println("false");	
		return	false;
	}

	
}






