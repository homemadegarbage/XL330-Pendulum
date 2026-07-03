# XL330 Wheeled Inverted Pendulum Robot

Atom Matrix (AtomS3) と DYNAMIXEL XL330-M077-T を使った2輪の倒立振子ロボットです。

IMU の角度を Kalman フィルタで推定し、左右の XL330 をカレント制御して倒立します。  
Atom Matrix (AtomS3) が Wi-Fi アクセスポイントになり、スマートフォンや PC のブラウザからジョイスティック操作と制御パラメータ調整ができます。