
/** 
* @file IcsBaseClass.h
* @brief ICS3.5/3.6 bace library header file
* @author Kondo Kagaku Co.,Ltd.
* @date 2017/12/27
* @version 2.1.0
* @copyright Kondo Kagaku Co.,Ltd. 2020

* @mainpage IcsClassの概要
* このライブラリは近藤科学製ロボット用サーボ(KRSシリーズ)を動かすためのライブラリです。<br>
* ICS3.5およびICS3.6に対応しています。<br>
* 現行では、Arduino製品で使えるように設計しています。<br>
* 使い方および詳細は、下記弊社HPをご覧ください。<br>
* <A HREF="http://kondo-robot.com/">http://kondo-robot.com/</A><br>
* 不具合等ありましたら、弊社HPを参照にご連絡ください。<br>

*@par	変更履歴
*@par 2020/02/20	Ver 2.1.0
* - keywords.txtのgetStrをgetStrcに変更
* - synchronizeがprotectedになっていたのをpublicに変更 //外部で使いたかったため
* - IDコマンドの実装
*@par 2017/12/27	ver 2.0.0
* - ICSClassを通信部分を分離。IcsBaseClassを作成。
* - Serialを使用する「IcsHardSerialClass」と
* <br> SoftSerialを使用する「IcsSoftSerialClass」をIcsBaseClassから派生させる
* - SoftSerialはパリティが使えずタイミングが合わなかったので、「KoCustomSoftSerial」を作成
*@par 2016/12/27　ver	1.0.0
* - 初回配布	
**/



#ifndef __ics_Base_Servo_h__
#define __ics_Base_Servo_h__
#include "Arduino.h"


  //KRR KRC-5FH ボタン定義////////
/**
 * @enum KRR_BUTTON
 * @brief KRRが受信するボタンデータの定義
 * @brief 同時押しの場合は各データの論理和をとります
 */  
enum KRR_BUTTON : unsigned short
{
  KRR_BUTTON_NONE = 0x0000, ///< 何も押されていない
    
  //左パッド
  KRR_BUTTON_UP        =  0x0001,  ///< ↑ 
  KRR_BUTTON_DOWN      =  0x0002,  ///< ↓ 
  KRR_BUTTON_RIGHT     =  0x0004,  ///< → 
  KRR_BUTTON_LEFT      =  0x0008,  ///< ← 
    
  //右パッド
  KRR_BUTTON_TRIANGLE  =  0x0010,  ///< △ 
  KRR_BUTTON_CROSS     =  0x0020,  ///< × 
  KRR_BUTTON_CIRCLE    =  0x0040,  ///< ○ 
  KRR_BUTTON_SQUARE    =  0x0100,  ///< □ 
  
  KRR_BUTTON_S1        =  0x0200,  ///< シフト1 左手前 
  KRR_BUTTON_S2        =  0x0400,  ///< シフト2 左奥 
  KRR_BUTTON_S3        =  0x0800,  ///< シフト3 右手前 
  KRR_BUTTON_S4        =  0x1000,  ///< シフト4 右奥 

  KRR_BUTTON_FALSE = 0xFFFF ///< エラー値(受信失敗等)
};



//IcsBaseClassクラス///////////////////////////////////////////////////
/**
* @class IcsBaseClass
* @brief 近藤科学のICS 3.5/3.6 サーボモータをマイコン経由で動作させるための基クラス
* @brief Arduino用にHardwareSerial,SoftwareSerialが使えるように基クラスを作成
**/
class IcsBaseClass 
{
  //固定値(公開分)
  public:
  //サーボID範囲 ////////////////////////////////////
  static constexpr int MAX_ID = 31;   ///< サーボIDの最大値
  static constexpr int MIN_ID = 0;    ///< サーボIDの最小値

  //サーボ最大最小リミット値

  static constexpr int MAX_POS = 11500;  ///<サーボのポジション最大値

  static constexpr int MIN_POS = 3500;   ///<サーボのポジション最小値
  
  static constexpr int ICS_FALSE = -1;  ///< ICS通信等々で失敗したときの値

  //固定値(非公開分)
  protected :

  static constexpr float ANGLE_F_FALSE = 9999.9; ///< 角度計算時、範囲内に入ってない場合は999.9にする(負側の場合はマイナスをつける)
  static constexpr int   ANGLE_I_FALSE = 0x7FFF; ///< 角度計算時、範囲内に入ってない場合は0x7FFFにする(負側の場合はマイナスをつける)

  //各パラメータ設定範囲
  static constexpr int MAX_127 = 127;   ///< パラメータの最大値
   
  static constexpr int MAX_63 = 63;     ///< パラメータの最大値(電流値)

  static constexpr int MIN_1 = 1;       ///< パラメータの最小値

  //  static const float MAX_DEG = 135.0;
  static constexpr float MAX_DEG = 180.0; ///< 角度の最大値

  //  static const float MIN_DEG = -135.0; 
  static constexpr float MIN_DEG = -180.0;  ///< 角度の最大値

  //static const int MAX_100DEG = 13500;
  static constexpr int MAX_100DEG = 18000;  ///< 角度(x100)の最大値
  
  //static const int MIN_100DEG = -13500;
  static constexpr int MIN_100DEG = -18000; ///< 角度(x100)の最小値
  
  //クラス内の型定義
  public:

  //コンストラクタ、デストラクタ
  public:
	//コンストラクタ(construncor)

  //変数
  public:


  protected : 

  //関数

  //データ送受信
  public:
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
	* @attention この関数は外部に書く事
	**/
     virtual bool synchronize(byte *txBuf, byte txLen, byte *rxBuf, byte rxLen);
   
  //servo関連
  public:

      //サーボ位置決め設定
      int setPos(byte id,unsigned int pos);    //目標値設定
      int setFree(byte id);    //サーボ脱力＋現在値読込
      
      //各種パラメータ書込み
      int setStrc(byte id, unsigned int strc);    //ストレッチ書込 1～127  1(弱）  <=>    127(強）
      int setSpd(byte id, unsigned int spd);      //スピード書込   1～127  1(遅い) <=>    127(速い)
      int setCur(byte id, unsigned int curlim);   //電流制限値書込 1～63   1(低い) <=>    63 (高い)
      int setTmp(byte id, unsigned int tmplim);   //温度上限書込   1～127  127(低温） <=> 1(高温) 
      //各種パラメータ読込み
      int getStrc(byte id);  //ストレッチ読込    1～127  1(弱） <=>     127(強）
      int getSpd(byte id);   //スピード読込      1～127  1(遅い)<=>     127(速い)
      int getCur(byte id);   //電流値読込        63←0 | 64→127
      int getTmp(byte id);   //現在温度読込      127(低温）<=>　0(高温)
      int getPos(byte id);   //現在位置読込　    ※ICS3.6以降で有効

      int getID();
      int setID(byte id);

  protected : 
      //サーボIDリミット
      byte idMax(byte id);

      ////サーボ可動範囲　パラメータ範囲　リミット設定
      bool maxMin(int maxPos, int minPos, int val);
      
  //角度関連
  public:    
      //角度変換 POSから角度へ変換
      static  int degPos(float deg);
      //角度変換 角度からPOSへ変換
      static float posDeg(int pos);

      //角度変換 x100 POSから角度へ変換
      static int degPos100(int deg);
      //角度変換 x100 角度からPOSへ変換
      static int posDeg100(int pos);

    //KRR関連
    public:
      //KRRからボタンデータ受信
      unsigned short getKrrButton();

      //KRRからPAアナログデータ受信
      int getKrrAnalog(int paCh);

      //KRRから全データ受信
      bool getKrrAllData(unsigned short *button,int adData[4]);

};

#endif
