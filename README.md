# README

## Environments
* Ubuntu 18.04
* ROS melodic

## How to build

```bash
  $ mkdir -p ws/src
  $ cd ws
  $ catkin init
  $ wget https://raw.githubusercontent.com/chikuta/premaidai_ros_bridge/master/premaidai_ros_bridge.rosinstall .rosinstall
  $ rosinstall .
  $ catkin build
```

## How to launch

### サーボをOFFにしてPremaidAIの状態表示

プリメイドAIの電源を入れて以下のコマンドを実行します。

```bash
  $ cd ws
  $ source devel/setup.bash
  $ roslaunch premaidai_ros_bridge display.launch serial_port:=/dev/frcomm0 servo_off_mode:=true
```

### rvizでPremaidAIを操作

```bash
  $ cd ws
  $ source devel/setup.bash
  $ roslaunch premaidai_ros_bridge rviz_control.launch serial_port:=/dev/frcomm0
```

<img src="https://raw.githubusercontent.com/wiki/chikuta/premaidai_ros_bridge/images/rviz_control.gif" width=50%>

## How to connect

プリメイドAIの電源をつけたあとに以下のコマンドでBD Adressを取得します。
プリメイドAIのBlueToothは `RNBT-XXXX` と認識されるので、このBD Addressをメモします。
ここでは `00:06:66:84:XX:XX` として表記します。

```bash
  $ hcitool scan

  Scanning ...
    00:06:66:84:XX:XX	RNBT-XXXX
```

次に接続時に使用するチャンネルを取得します。
出力される `RFCOMM` の `Channel` をメモします。
環境によって違いが出ると思うので以後 `${CHANNEL}` と表記します。

```bash
  $ sdptool records 00:06:66:84:XX:XX

  Service Name: RNI-SPP
  Service RecHandle: 0x10000
  Service Class ID List:
    "Serial Port" (0x1101)
  Protocol Descriptor List:
    "L2CAP" (0x0100)
    "RFCOMM" (0x0003)
      Channel: 1
  Language Base Attr List:
    code_ISO639: 0x656e
    encoding:    0x6a
    base_offset: 0x100
```

`/etc/bluetooth/rfcomm.conf` に以下を記述して再起動します。

```text
rfcomm0 {
    bind yes;
    device 00:06:66:84:XX:XX;
    channel ${CHANNEL};
    comment "serial port";
}
```

以下のコマンドを入力してプリメイドAIを接続します。

```bash
  $ sudo rfcomm release ${CHANNEL}
  $ sudo rfcomm bind rfcomm0 00:06:66:84:XX:XX ${CHANNEL}
```

この場合、 `/dev/rfcomm0` がプリメイドAIと接続がされたシリアルポートになります。

## Refers
* [楽しく遊ぶプリメイドAI 解析メモ](https://docs.google.com/spreadsheets/d/1c6jqMwkBroCuF74viU_q7dgSQGzacbUW4mJg-957_Rs/edit#gid=2102495394)

## LICENSE
MIT