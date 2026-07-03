# XL330 Wheeled Inverted Pendulum Robot

Atom Matrix (AtomS3) と DYNAMIXEL XL330-M077-T を使った2輪の倒立振子ロボットです。

IMU の角度を Kalman フィルタで推定し、左右の XL330 をカレント制御して倒立します。  
Atom Matrix (AtomS3) が Wi-Fi アクセスポイントになり、スマートフォンや PC のブラウザからジョイスティック操作と制御パラメータ調整ができます。

## Contents

- `3Dmodel/` - 実機ロボット用の3Dプリントモデル
- `Arduino/` - Atom Matrix (AtomS3) 用スケッチ

## Arduinoコード環境・必要ライブラリ
- Arduino IDEバージョン：ver. 1.8.19
- ESP32ライブラリ：ver. 2.0.13
- [M5Atomライブラリ](https://github.com/m5stack/M5Atom/tree/master)：ver. 0.1.3
- [Kalman Filter Library](https://github.com/TKJElectronics/KalmanFilter) ：ver. 1.0.2
- [Dynamixel2Arduino](https://github.com/ROBOTIS-GIT/dynamixel2arduino)：ver. 0.8.1
