# Open Drone Project(ODP)
## 概要
このプロジェクトは、Arduinoプラットフォーム上で開発可能なドローン制御システムの構築を目的としています。
***
## 電源仕様
### USBバスパワー給電時
- USB Type-C(USB2.0)で供給を行う
- バッテリーの充電が行われる
- バッテリー給電からUSBバスパワー給電に自動的に切り替わる
- システム全体の消費電流が750mAを超えるとリセッタブルヒューズの抵抗値が上がる
- ESP32-WROOM-32とバッテリー残量IC以外のセンサが起動する
- モーターが回転しないようソフト側で制限をかける
### バッテリー給電時
- 供給能力1000mA 3Cのリチウムイオン二次電池から供給を行う
- モーター含むシステム全体に電力を供給する
- すべての負荷はバッテリー残量ICを通過して電力が供給される
- 全モーターの消費電流が3A(仮)を超えたらソフト側で制限をかける
### バッテリー充電
- バッテリー充電はMCP73831-2ATI/OTを用いて行う
- 最大充電電流はPROG-GND間抵抗5kΩで最大200mAに設定する
#### MCP73831-2ATI/OTの最大充電電流の設定([公式データシート](https://www.microchip.jp/docs/DS21984B_JP.pdf)より)
```
IREG=最大充電電流
PROG=最大充電電流設定用抵抗

IREG=1000[V]/PROG[kΩ]
```
原則`最大充電電流[mA]≦バッテリー容量[mAh]`で設定する
### 消費電力
#### USBバスパワー給電時
|負荷 |消費電流 |
|---|---|
|[ESP32-WROOM-32D](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)|500mA|
|[MCP73831-2ATI/OT](https://www.microchip.jp/docs/DS21984B_JP.pdf)|200mA|
|[BQ27441-G1](http://www.tij.co.jp/jp/lit/ds/symlink/bq27441-g1.pdf)|0mA|
|[FT231XS](https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT231X.pdf)|8mA|
|[BMX055](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMX055-DS000.pdf)|8mA|
|[AE-VL53L0X](https://www.st.com/content/ccc/resource/technical/document/datasheet/group3/b2/1e/33/77/c6/92/47/6b/DM00279086/files/DM00279086.pdf/jcr:content/translations/en.DM00279086.pdf)|19mA|
|[B0067](http://www.arducam.com/downloads/shields/ArduCAM_Mini_2MP_Camera_Shield_DS.pdf)|70mA|
|[モーター](http://www.vibration-motor.com/products/download/Q7AL2BX180003.pdf)|0mA|
|**合計**|805mA|
#### バッテリー給電時
|負荷 |消費電流 |
|---|---|
|[ESP32-WROOM-32D](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)|500mA|
|[MCP73831-2ATI/OT](https://www.microchip.jp/docs/DS21984B_JP.pdf)|0mA|
|[BQ27441-G1](http://www.tij.co.jp/jp/lit/ds/symlink/bq27441-g1.pdf)|0.097mA|
|[FT231XS](https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT231X.pdf)|8mA|
|[BMX055](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMX055-DS000.pdf)|8mA|
|[AE-VL53L0X](https://www.st.com/content/ccc/resource/technical/document/datasheet/group3/b2/1e/33/77/c6/92/47/6b/DM00279086/files/DM00279086.pdf/jcr:content/translations/en.DM00279086.pdf)|19mA|
|[B0067](http://www.arducam.com/downloads/shields/ArduCAM_Mini_2MP_Camera_Shield_DS.pdf)|70mA|
|[モーター](http://www.vibration-motor.com/products/download/Q7AL2BX180003.pdf)|1130.4mA|
|**合計**|1739.097mA|