# UR controllers

# COUNTION! THIS REPOS IS UNDER HEAVY DEVELOPMENT!

## Controllers

### scaled_pos_joint_traj_controller(Default motion controller)

`control_msgs/FollowJointTrajectory` 用のコントローラ．MoveItで使用される．

### joint_group_vel_controller

`velocity_controllers/JointGroupVelocityController`

トピックインターフェイスを介して関節速度を直接コマンドします.

このコントローラは、ジョイントスペースでサーボを行う場合に便利です.

### twist_controller

joint_group_vel_controllerと同様に、このコントローラではトピックインターフェースを介してロボットに直接速度を送信することができます。ただし、このコントローラはデカルトのTCPツイスト（直線速度と角速度）を想定しています。これは、ビジュアルサーボや遠隔操作のようなデカルトサーボアプリケーションで有用です。

このコントローラは、ロボットのキネマティクスにおける構成の変更をチェックしないことに注意してください。ツイストコマンドは、ロボットへの実行のために送信されます。そのため、ロボットの動作空間から離れるときや、強制的に構成を変更させるときなど、指令された動作が保護停止になることがあります．

----

### forward_joint_traj_controller

このコントローラは、control_msgs/FollowJointTrajectoryインターフェースを実装しています。ROS側で軌道を補間する代わりに、これは完全な軌道をロボットに転送し、加速度と速度の計算をロボットに任せて実行させます。そのため、ティーチペンダントで設定したものに最も近い動作になります。scaled_pos_joint_traj_controllerと少しだけ異なる起動を実行します．各ウェイポイントはブレンドされます．



### forward_cartesian_traj_controller

このコントローラは cartesian_control_msgs/FollowCartesianTrajectory インターフェースを実装しています。ROS側で軌道を補間する代わりに、これは完全な軌道をロボットに転送し、加速度と速度の計算をロボットに任せて実行させます。そのため、ティーチペンダントで設定したものに最も近い動作になります.



## Switch the controller

コントローラを切り替えるためには，`controller_manager`で用意されている方法を利用します．例えば，デフォルトの`scaled_pos_joint_traj_controller`から`joint_group_vel_controller`へ切り替えるには，下記の内容でros serviceをコールします．

```
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
stop_controllers: ['scaled_pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.0"
```

## Example analyze

- `joint_group_vel_controller` と`twist_controller` は`joint trajectory controller`と競合してしまうので，実行するときは，他のコントローラが実行されていない状態にする必要があります．



### controller_manager/switch_controller(controller_manager/SwitchController)

```c++
#include <controller_msgs/SwitchController.h>
```



サービスリクエストには、開始するコントローラ名のリスト、停止するコントローラ名のリスト、および厳密性(BEST_EFFORTまたはSTRICT)を示すintが含まれています。STRICTは、何か問題が発生した場合(無効なコントローラ名、起動に失敗したコントローラなど)、切り替えに失敗し、no-opとなることを意味します。BEST_EFFORTは、コントローラで何か問題が発生した場合でも、サービスが残りのコントローラの起動/停止を試みることを意味します。サービスのレスポンスには、成功または失敗を示すブーリアンが含まれます。停止または起動するコントローラのリストには、空のリストを指定することができます。

SwitchControllerサービスを使用すると、controller_manager制御ループの1つのタイムステップで、複数のコントローラを停止したり、複数のコントローラを起動したりすることができます。

コントローラを切り替えるには、以下を指定します。
開始するコントローラ名のリスト
停止するコントローラ名のリスト
厳密さ（BEST_EFFORTまたはSTRICT)
STRICTは、何か問題があった場合(コントローラ名が無効、コントローラの起動に失敗したなど)、切り替えに失敗することを意味します。
BEST_EFFORT は、コントローラに何か問題が発生した場合でも、サービスは残りのコントローラを起動/停止しようとします。そして，ハードウェアの準備ができ次第、コントローラを起動します。すべてのインターフェイスの準備が整うまで待ちます。それ以外の場合は、保留中のコントローラを中止するまでのタイムアウトを秒で指定します。無限の場合は0

戻り値 "ok "は、コントローラの切り替えに成功したか否かを示します。 成功の意味は、指定された厳密性に依存します。

## Scheme of using twist controller

1. Load `twist_controller` on `controller_manager` using `rosservice call /controller_manager/load_controller name: twist_controller`
2. Switch controller to `twist_controller` using `rosservice call /controller_manager/switch_controller`
3. You can find three topic
   - `twist_controller/command` represents TCP velocity command.
   - `twist_controller/parameter_description` represents  `dynamic_reconfigure/config_description`
   - `twist_conroller/parameter_updates` represents `dynemic_reconfigure/config`
4. You can publish `twist_controller/command`

## Dualsense mapping

## StertUP

```
roslaunch irex_demo enable_robot.launch
```

In teaching pendant 

- Run `External Control` 
