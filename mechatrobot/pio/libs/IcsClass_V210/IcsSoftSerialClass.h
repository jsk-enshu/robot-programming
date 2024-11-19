
/** 
*  @file IcsSoftSerialClass.h
* @brief ICS3.5/3.6 arduino library use SoftwareSeria header file
* @author Kondo Kagaku Co.,Ltd.
* @date 2020/02/20
* @version 2.1.0
* @copyright Kondo Kagaku Co.,Ltd. 2017



**/

#ifndef _ics_SoftSerial_Servo_h_
#define _ics_SoftSerial_Servo_h_

//#include <Arduino.h>
#include <IcsBaseClass.h>
#include <KoCustomSoftSerial.h>


//IcsClassクラス///////////////////////////////////////////////////
/**
* @class IcsSoftSerialClass
* @brief 近藤科学のKRSサーボをArduinoのSoftwareSerialからアクセスできるようにしたクラス
* @brief IcsBaseClassからの派生して作られている
* @brief SoftwareSerialは高速通信は難しいためKoCustomSoftSerialを使用する
**/
class IcsSoftSerialClass : public IcsBaseClass
{
  //クラス内の型定義
  public:

  //コンストラクタ、デストラクタ
  public:
      //コンストラクタ(construncor)

	IcsSoftSerialClass();

	IcsSoftSerialClass(byte rxPin,byte txPin,byte enpin);
	IcsSoftSerialClass(byte rxPin,byte txPin,byte enpin, long baudrate, int timeout);
	IcsSoftSerialClass(KoCustomSoftSerial *koSoftSerial,byte enpin);
	IcsSoftSerialClass(KoCustomSoftSerial *koSoftSerial,byte enpin, long baudrate, int timeout);
      
      //デストラクタ(destruntor)
      ~IcsSoftSerialClass();
  
  //変数
  public:


  protected: 
	KoCustomSoftSerial *icsSoftSerial;  ///<arudinoのシリアル型のポインタを格納
	byte g_enpin;         ///<イネーブルピン(送受信を切り替える)のピン番号を格納しておく変数
	int		g_timeout = 1000;				///<タイムアウトの設定を格納しておく変数
	long	g_baudrate = 115200;			///<通信速度の設定を格納しておく変数

  //関数
  //通信初期化
  public:
      virtual bool begin();
      virtual bool begin(long baudrate,int timeout);
      virtual bool begin(KoCustomSoftSerial *serial,byte enpin,long baudrate,int timeout);


  //イネーブルピンの処理
  protected : 
  	/**
	*	@brief enPinに割り当てられているピンをHにする
	**/
	inline void enHigh(){digitalWrite(g_enpin, HIGH);}
	/**
	*	@brief enPinに割り当てられているピンをLにする
	**/
	inline void enLow(){digitalWrite(g_enpin, LOW);}

  //データ送受信
  public:
      virtual bool synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen);
   
  //servo関連	//すべていっしょ
  public:


};

#endif 