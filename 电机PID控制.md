电机PID控制

## 1、PID基础

PID即：Proportional（比例）、Integral（积分）、Differential（微分）的缩写。

PID是经典的闭环控制算法，具有原理简单，易于实现，适用面广，控制参数相互独立，参数的选定比较简单等优点。

凡是需要将某一个物理量“保持稳定”的场合（比如维持平衡，稳定温度、转速等），PID都会派上大用场。

![](file:///C:/Users/s50027046/AppData/Roaming/marktext/images/2022-08-04-17-54-45-image.png?msec=1661756940812)

### 1.1 PID算法分类

PID算法可分为**位置式PID**与**增量式PID**两大类。

在实际的编程应用中，需要使用离散化的PID算法，以适用计算机的使用环境，下面以电机转速控制为例，来看一下两种PID算法的基本原理。

#### 1.1.1 位置式PID

位置式PID是当前系统的实际位置，与想要达到的预期位置的偏差，进行PID控制

![](file:///C:/Users/s50027046/AppData/Roaming/marktext/images/2022-08-04-19-57-16-image.png?msec=1661756940829)

- 比例P：e(k) 此次误差
- 积分I：∑e(i) 误差的累加
- 微分D：e(k) - e(k-1) 此次误差-上次误差

因为有误差积分 ∑e(i)，一直累加，也就是当前的输出u(k)与过去的所有状态都有关系。

**位置式PID算法的伪代码如下**：

```c
//位置式PID（伪代码）
previous_err = 0;
integral = 0;
loop: //根据目标值与测量值(如电机的设定速度与读到的编码器转后后的速度)，循环计算更新输出值（如PWM）
    error = setpoint - measured_value;          /*误差项：目标值-测量值*/
    integral += error * dt;                     /*积分项：误差项的累计*/
    derivative = (error - previous_error) / dt; /*微分项：误差的变化率*/
    output = Kp*error + Ki*integral + Kd*derivative; /*三项分别乘以PID系数即为输出*/
    previous_err = error; //更新误差
    wait(dt); //等待固定的计算周期
    goto loop;
```

#### 1.1.2 增量式PID

![](file:///C:/Users/s50027046/AppData/Roaming/marktext/images/2022-08-04-20-00-14-image.png?msec=1661756940820)

- 比例P：e(k) - e(k-1) 此次误差-上次误差
- 积分I：e(k) 此次误差d
- 微分D：e(k) - 2e(k-1)+e(k-2) 这次误差-2×上次误差+上上次误差

注意增量式PID首先计算的是Δu(k)，然后与上次的输出相加，才是此次的输出结果。增量式PID没有误差累加，控制增量Δu(k)的确定仅与最近3次的采样值有关。

**增量式PID算法的伪代码如下**：

```c
//增量式PID（伪代码）
previous02_error = 0; //上上次偏差
previous01_error = 0; //上次偏差
integral = 0; //积分和
pid_out = 0; //pid增量累加和
loop:
    error = setpoint − measured_value;  /*误差项：目标值-测量值*/
    proportion = error - previous01_error;        /*比例项：误差项-上次偏差*/
    integral = error * dt;                        /*积分项：误差项的累计*/
    derivative = (error − 2*previous01_error + previous02_error) / dt;
     /*微分项：上次误差与上上次误差的变化率, 或写成:derivative = ((error − 
     previous01_error)/dt - (previous01_error - previous02_error)/dt )*/
    pid_delta = Kp × proportion + Ki × integral + Kd × derivative; //计算得到PID增量
    pid_out = pid_out + pid_delta; //计算最终的PID输出
​
    previous02_error = previous01_error; //更新上上次偏差
    previous01_error = error; //更新上次偏差
    wait(dt); //等待固定的计算周期
    goto loop;
```

### 1.2 PID各项的作用

以这个弹簧为例（假设没有重力，只有空气阻力），先是在**平衡位置上(目标位置)**，拉它一下，然后松手，这时它会震荡起来。

                                                                    

#### 1.2.1 P 比例

**P就是比例**的意思。这里就类比弹簧的弹力（回复力）：`F=k*Δx`

- 当物块距离平衡位置越远时，弹力越大，反之，离平衡位置越近，力越小。
- 当物块位于平衡位置上方时，弹性向下，当物块位于平衡位置下方时，弹性向上，即弹力总是使物块朝平衡位置施力。

#### 1.2.2 D 微分/求导/变化率

只有P控制，物块一直在上下震荡，整个系统不是特别稳定。

这是因为空气阻力太小，想象一下整个把它放到水里，物块应该很快会静止下来。这时因为**阻力**的作用。

**D的作用就相当于阻力**：

- 它与变化速度（单位时间内的变化量）有关，变化的越大，它施加的阻力也就越大
- 它的方向与目标值无关，比如，*当物块从下到上经过平衡位置时*，它的方向一直是朝下，即先是阻止物块靠近平衡位置，再是阻止物块远离平衡位置（对比P的作用，始终阻止物块远离平衡位置）
- 它的作用就是减小系统的超调量了（减少系统在平衡位置震荡）

#### 1.2.3 I 积分/误差累积

有了P的动力和D的阻力，这个物块就可以较快的稳定下来了，那I的作用是什么呢？

想象一下，如果有其它外力的影响，在某一时刻，物块将要到达平衡位置时，恰好P的动力与外力(与P的作用方向相反的恒定力)抵消，则之后物块将停在此处附近（因为此时D的力也趋近0，并很快变为0），一直到达不了平衡位置。

这时，**I的误差积分**作用就很有必要了：

- 它计算的误差的累计，只要有误差，它就一直增加，开始可能很小，但只要没要到达平衡位置，该值就会越来越大
- 它的作用就是消除系统的静态误差了

### 1.3 PID参数整定

实际应用，进行PID参数调节时，一般使用试凑法，PID参数整定口诀如下：

> 参数整定找最佳，从小到大顺序查，  
> **先是比例后积分，最后再把微分加**，  
> 曲线振荡很频繁，**比例**度盘要放大，  
> 曲线漂浮绕大湾，**比例**度盘往小扳，  
> 曲线偏离回复慢，**积分**时间往下降，  
> 曲线波动周期长，**积分**时间再加长，  
> 曲线振荡频率快，先把**微分**降下来，  
> 动差大来波动慢，**微分**时间应加长，  
> 理想曲线两个波，前高后低4比1，  
> 一看二调多分析，调节质量不会低。

## 2、电机PID速度控制

上面介绍了PID的基础知识，接下来就使用位置式PID来实现对直流电机转速的控制

### 2.1 程序

**自定义结构体**

```c
typedef struct
{
    float target_val;  //目标值
    float error;       //偏差值
    float error_last;  //上一个偏差值
    float Kp,Ki,Kd;    //比例、积分、微分系数
    float integral;    //积分值
    float output_val;  //输出值
}PID;
```

#### 2.1.1 PID算法实现(位置式PID)

```c
float PID_realize(float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid.error = pid.target_val — actual_val;

    /*积分项*/
    pid.integral += pid.error;

    /*PID算法实现*/
    pid.output_val = pid.Kp * pid.eeror +
                     pid.Ki * pid.integral +
                     pid.Kd * (pid.error — pid.error_last);

    /*误差传递*/
    pid.error_last = pid.error;

    /*返回当前实际值*/
    return pid.output_val;
}
```

#### 2.1.2 周期调用PID计算

```c
//周期定时器的回调函数
void AutoReloadCallback()
{
    int sum = 0;      //编码器值（PID输入）
    int res_PWM = 0;  //PWM值（PID输出）

    /*读取编码器测量的速度值*/
    sum = read_encoder();

    /*进行PID运算，得到PWM输出值*/
    res_PWM = PID_realize(sum);

    /*根据PWM值控制电机转动*/
    set_motor_rotate(res_PWM);

    /*给上位机通道1发送实际值*/
    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &sum, 1);
}
```
