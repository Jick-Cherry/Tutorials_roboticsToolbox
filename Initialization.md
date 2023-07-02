# 初始化

## 开始
启动图形化交互命令：roblocks
启动教程：rtbdemo
## 创建模型

串联连杆操纵器包括一系列连杆。每个链路由四个DH参数描述。
让我们定义一个简单的2连杆操纵器。
L1 = Link('d', 0, 'a', 1, 'alpha', pi/2) 
L1 = 
Revolute(std): theta=q, d=0, a=1, alpha=1.5708, offset=0
The Link object we created has a number of properties
>> L1.a
ans =  1

and we determine that it is a revolute joint旋转关节
>> L1.isrevolute
 

ans =
  logical
    1

For a given joint angle, say q=0.2 rad, we can determine the link transform
matrix
>> L1.A(0.2)
 
 

ans = 
    0.9801         0    0.1987    0.9801
    0.1987         0   -0.9801    0.1987
         0         1         0         0
         0         0         0         1

Now we need to join these into a serial-link robot manipulator

>> bot = SerialLink([L1 L2], 'name', 'my robot')
 
 
bot = 
 
my robot:: 2 axis, RR, stdDH, slowRNE                            
+---+-----------+-----------+-----------+-----------+-----------+
| j |     theta |         d |         a |     alpha |    offset |
+---+-----------+-----------+-----------+-----------+-----------+
|  1|         q1|          0|          1|     1.5708|          0|
|  2|         q2|          0|          1|          0|          0|
+---+-----------+-----------+-----------+-----------+-----------+

The displayed robot object shows a lot of details.  It also has a number of
properties such as the number of joints
>> bot.n
ans = 2
     
     
Given the joint angles q1 = 0.1 and q2 = 0.2 we can determine the pose of the
robot's end-effector

>> bot.fkine([0.1 0.2])

ans = 
    0.9752   -0.1977    0.0998      1.97
    0.0978   -0.0198   -0.9950    0.1977
    0.1987    0.9801         0    0.1987
         0         0         0         1

which is referred to as the forward kinematics of the robot.  This, and the
inverse kinematics are covered in separate demos.

Finally we can draw a stick figure of our robot

>> bot.plot([0.1 0.2])


## 雅可比矩阵
雅可比和微分运动演示

差分运动可以用具有元素的6元素矢量来表示
[dx dy dz drx dry drz]
其中前3个元素是差分平移，后3个元素
是差动旋转。当处理无限旋转时，顺序变得不重要。微分运动可以写成
就复合变换而言
transl（dx，dy，dz）*trotx（drx）*troty（dry）*trotz（drz）

但更直接的方法是使用函数diff2tr（）

>>D=[1.2 0-.2.1.1]'；
>>delta2tr(D)
 

ans =

    1.0000   -0.1000    0.1000    0.1000
    0.1000    1.0000    0.2000    0.2000
   -0.1000   -0.2000    1.0000         0
         0         0         0    1.0000

更常见的是，了解一个坐标系中的差分运动如何出现在另一坐标系中是有用的。如果第二帧由变换表示

More commonly it is useful to know how a differential motion in one
coordinate frame appears in another frame.  If the second frame is
represented by the transform

>> T = transl(100, 200, 300) * troty(pi/8) * trotz(-pi/4);
then the differential motion in the second frame would be given by

>> DT = tr2jac(T) * D;
 

>> DT'
 

ans =

    0.0972    0.2014    0.0007   -0.2020    0.0972    0.0986

tr2jac（）计算了一个6x6雅可比矩阵，该矩阵将差分变化从第一帧转换到下一帧。
------

机械手的雅可比矩阵将微分关节坐标运动与微分笛卡尔运动联系起来；

dX=J（q）dQ

对于n关节机械手，机械手雅可比矩阵是一个6×n矩阵，用于许多机械手控制方案。

两个雅可比矩阵经常被使用，它们表示世界坐标系中的笛卡尔速度。我们将首先为机器人选择一个特定的关节角度配置

>> q = [0.1 0.75 -2.25 0 .75 0]
 

q =

    0.1000    0.7500   -2.2500         0    0.7500         0

and then compute the Jacobian in the world coordinate frame

>> J = p560.jacob0(q)
 

J =

    0.0746   -0.3031   -0.0102         0         0         0
    0.7593   -0.0304   -0.0010         0         0         0
         0    0.7481    0.4322         0         0         0
   -0.0000    0.0998    0.0998    0.9925    0.0998    0.6782
   -0.0000   -0.9950   -0.9950    0.0996   -0.9950    0.0681
    1.0000    0.0000    0.0000    0.0707    0.0000    0.7317


which we can see is 6x6 (since the robot has 6 joints)

Alternatively the Jacobian can be expressed in the T6 coordinate frame

>> J = p560.jacobe(q)
 

J =

    0.1098   -0.7328   -0.3021         0         0         0
    0.7481    0.0000    0.0000         0         0         0
    0.1023    0.3397    0.3092         0         0         0
   -0.6816         0         0    0.6816         0         0
   -0.0000   -1.0000   -1.0000   -0.0000   -1.0000         0
    0.7317    0.0000    0.0000    0.7317    0.0000    1.0000


Note the top right 3x3 block is all zero.  This indicates, correctly, that
motion of joints 4-6 does not cause any translational motion of the robot's
end-effector.
或者，雅可比可以在T6坐标系中表示

请注意，右上角的3x3块全部为零。这正确地表明

关节4-6的运动不会引起机器人末端执行器的任何平移运动。

Many control schemes require the inverse of the Jacobian.  The Jacobian
in this example is not singular
许多控制方案都需要雅可比矩阵的逆。这个例子中的雅可比不是奇异的
>> det(J)
ans =
   -0.0632

and may be inverted
并且可以转置
>> Ji = inv(J)
Ji =

    0.0000    1.3367   -0.0000    0.0000    0.0000         0
   -2.4946    0.6993   -2.4374   -0.0000   -0.0000         0
    2.7410   -1.2106    5.9125    0.0000    0.0000         0
    0.0000    1.3367   -0.0000    1.4671   -0.0000         0
   -0.2464    0.5113   -3.4751   -0.0000   -1.0000         0
   -0.0000   -1.9561    0.0000   -1.0734    0.0000    1.0000

A classic control technique is Whitney's resolved rate motion control
dQ/dt = J(q)^-1 dX/dt
where dX/dt is the desired Cartesian velocity, and dQ/dt is the required
joint velocity to achieve this.
一种经典的控制技术是Whitney的分解速率运动控制
dQ/dt=J（q）^-1 dX/dt
其中dX/dt是所需的笛卡尔速度，dQ/dt是所要求的关节速度来实现这一点。
>> vel = [1 0 0 0 0 0]'; % translational motion in the X direction

>> qvel = Ji * vel;

>> qvel'

ans =

    0.0000   -2.4946    2.7410    0.0000   -0.2464   -0.0000

This is an alternative strategy to computing a Cartesian trajectory
and solving the inverse kinematics.  However like that other scheme, this
strategy also runs into difficulty at a manipulator singularity where
the Jacobian is singular.

As already stated this Jacobian relates joint velocity to end-effector
velocity expressed in the end-effector reference frame.  We may wish
instead to specify the velocity in base or world coordinates.
We have already seen how differential motions in one frame can be translated
to another.  Consider the velocity as a differential in the world frame, that
is, d0X.  We can write

这是计算笛卡尔轨迹的另一种策略以及求解逆运动学。然而，与其他方案一样
这种策略在机械手奇异点也遇到了困难:雅可比矩阵是奇异的。

如前所述，雅可比将关节速度与末端执行器联系起来
末端执行器参考系中表示的速度。我们不妨
而是指定基准坐标或世界坐标中的速度。
我们已经看到了如何平移一帧中的差分运动
到另一个。将速度视为世界坐标系中的一个微分，即
即d0X。我们可以写作
d6X = Jac(T6) d0X

>> T6 = p560.fkine(q); % compute the end-effector transform

>> d6X = tr2jac(T6) * vel; % translate world frame velocity to T6 frame

>> qvel = Ji * d6X; % compute required joint velocity as before

>> qvel'
 

ans =

   -0.1334   -3.5391    6.1265   -0.1334   -2.5874    0.1953



Note that this value of joint velocity is quite different to that calculated
above, which was for motion in the T6 X-axis direction.

At a manipulator singularity or degeneracy the Jacobian becomes singular.
At the Puma's `ready' position for instance, two of the wrist joints are
aligned resulting in the loss of one degree of freedom.  This is revealed by
the rank of the Jacobian
>> rank( p560.jacobe(qr) )
 

ans =

     5

and the singular values are
>> svd( jacobe(p560, qr) )
 

ans =

    1.9066
    1.7321
    0.5660
    0.0166
    0.0081
    0.0000

When not actually at a singularity the Jacobian can provide information
about how `well-conditioned' the manipulator is for making certain motions,
and is referred to as `manipulability'.
A number of scalar manipulability measures have been proposed.  One by
Yoshikawa

>> p560.maniplty(q, 'yoshikawa')
 
Manipulability: translation 0.0927222, rotation 2.40266


is based purely on kinematic parameters of the manipulator.
Another by Asada takes into account the inertia of the manipulator which
affects the acceleration achievable in different directions.  This measure
varies from 0 to 1, where 1 indicates uniformity of acceleration in all
directions

>> p560.maniplty(q, 'asada')
 
Manipulability: translation 0.0642221, rotation 0.154985

Both of these measures would indicate that this particular pose is not well
conditioned.