#include "MotorControl.h"
#include "ParametersDefine.h"
#include "Senser.h"

// 计算方位角速度补偿值(前馈)
extern float GetFeedForwardAZ(void);

// 方位缓启动
float AZSlowStart(float kp, unsigned char time, float error, float maxspeed);

// 外推滤波
float ExtrapolateFilter(float value, float oldvalue)
{
    float err = 0;

    err = value - oldvalue;
    if (err > 300) {
        err -= 360;
    }
    if (err < -300) {
        err += 360;
    }

    if (fabs(err) > 10) {
        return oldvalue;
    } else {
        return value;
    }
}

// 方位位置环PID
float PositionControlAZ(float PstnPerset, float* PstnReal)
{
    float kp = 0.0, ki = 0.0, kd = 0.0, td = 0.0, fk = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0;
    float value_p_limit = 0, value_i_limit = 0;
    float error = 0, error_old = 0;
    float error_i = 0;
    float err_p = 0; //前馈使用，预置的差值
    float err[2];
    float value = 0;
    int i = 0;
    unsigned char time_200ms = 0.2 * _CTRCLK;
    int _lookHeadTime = 0;
    float accelerate = 0.0;
    float accelerateMax = 500;

    float value_i_old = MotoCtr.PIDAZ.I;
    float value_d_old = MotoCtr.PIDAZ.D;

    kp = P_AZ;
    ki = I_AZ / 100.0f;
    kd = D_AZ;
    fk = F_AZ;
    _lookHeadTime = ((int)(LHT_AZ * 100) / 100);

    //	kp = 35;
    //	ki = 0;
    //	kd = 0;
    //	fk = 0;
    //	_lookHeadTime = 30;

    td = 0.0;

    for (i = 0; i < 300; i++)
        MotoCtr.AZP[i] = MotoCtr.AZP[i + 1];
    MotoCtr.AZP[300] = PstnPerset;

    //防止外推时方位0与360转换出现问题
    if (PstnPerset > 270 || PstnPerset < 90) {
        for (i = (300 - _lookHeadTime * 2); i < 301; i++) {
            if (MotoCtr.AZP[i] > 180)
                MotoCtr.AZP_[i] = MotoCtr.AZP[i] - 360;
            else
                MotoCtr.AZP_[i] = MotoCtr.AZP[i];
        }
        MotoCtr.PIDAZ.LookHead = Extrapolate(MotoCtr.AZP_, _lookHeadTime, _CTRCLK, 300);
        if (MotoCtr.PIDAZ.LookHead < 0)
            MotoCtr.PIDAZ.LookHead += 360;
    } else {
        MotoCtr.PIDAZ.LookHead = Extrapolate(MotoCtr.AZP, _lookHeadTime, _CTRCLK, 300);
    }

    MotoCtr.PIDAZ.LookHeadBeforeL = MotoCtr.PIDAZ.LookHead;
    MotoCtr.PIDAZ.LookHead = ExtrapolateFilter(MotoCtr.PIDAZ.LookHeadBeforeL, PstnPerset);

    MotoCtr.PIDAZ.err = _180convert(MotoCtr.PIDAZ.LookHead - PstnReal[10]);

    MotoCtr.PIDAZ.err = _10convert(MotoCtr.PIDAZ.err);
    error_i = _10convert(MotoCtr.PIDAZ.err);

    value_p_limit = 90;
    value_p = AZSlowStart(kp, time_200ms, MotoCtr.PIDAZ.err, value_p_limit);

    value_i_limit = 90;
    MotoCtr.PIDAZ.integral += error_i;
    if (ki == 0)
        MotoCtr.PIDAZ.integral = 0;

    value_i = uLimit(ki * MotoCtr.PIDAZ.integral, value_i_limit);

    value_d = uLimit(kd * (MotoCtr.PIDAZ.err - MotoCtr.PIDAZ.err_last), 90);
    MotoCtr.PIDAZ.err_last = MotoCtr.PIDAZ.err;

    //前馈补偿
    //value_k = fk * GetFeedForwardAZ();
    err_p = PstnPerset - MotoCtr.PIDAZ.PLast;
    if (err_p > 300) {
        err_p = err_p - 360;
    }
    if (err_p < -300) {
        err_p = err_p + 360;
    }
    //	value_k = fk * (PstnPerset - MotoCtr.PIDAZ.PLast) * _CTRCLK;
    value_k = fk * err_p * _CTRCLK;
    MotoCtr.PIDAZ.PLast = PstnPerset;

    value = uLimit(value_p + value_i + value_d + value_k, 90);
    MotoCtr.PIDAZ.ValueBeforeL = value;
    if (HeadCheckFlag == 0) {
        accelerateMax = ACCMAXAZ;
        //加速度限制
        accelerate = value - Senser.SpeedAZ;
        if (fabs(accelerate) > accelerateMax) {
            if (accelerate > 0)
                value = Senser.SpeedAZ + accelerateMax;
            else
                value = Senser.SpeedAZ - accelerateMax;
        }
    }
    value = uLimit(value, 90);

    MotoCtr.PIDAZ.Value_Last = value;
    MotoCtr.PIDAZ.P = value_p;
    MotoCtr.PIDAZ.I = value_i;
    MotoCtr.PIDAZ.D = value_d;
    MotoCtr.PIDAZ.K = value_k;
    MotoCtr.PIDAZ.Value = value;
    return value;
}

// 方位位置环PID，指向
float PositionControlAZPoint(float PstnPerset, float* PstnReal)
{
    float kp = 0.0, fk = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0;
    float error = 0, error_old = 0;
    float err[2];
    float value = 0;
    int i = 0;
    int time_200ms = 0.2 * _CTRCLK;
    float accelerate = 0, accelerateMax = 0;

    kp = 7.0;

    fk = F_AZ;

    for (i = 0; i < 300; i++)
        MotoCtr.AZP[i] = MotoCtr.AZP[i + 1];
    MotoCtr.AZP[300] = PstnPerset;

    error = PstnPerset - PstnReal[10];
    error_old = PstnPerset - PstnReal[9];

    MotoCtr.PIDAZ.LookHead = PstnPerset;

    err[1] = _180convert(error);
    err[0] = _180convert(error_old);

    if (MotoCtr.Timer.az < time_200ms) {
        MotoCtr.Timer.az++;
        if (kp != 0) {
            if (fabs(error) > 10 / kp)
                value_p = uLimit(kp * err[1], 50) * MotoCtr.Timer.az / time_200ms;
            else
                value_p = uLimit(kp * err[1], 50);
        } else
            value_p = 0;
    } else {
        value_p = uLimit(kp * err[1], 50);
    }

    value_d = 0;
    value_i = 0;
    MotoCtr.PIDAZ.integral = 0;

    value_k = fk * GetFeedForwardAZ();
    value_k = fk * (MotoCtr.PIDAZ.PLast - PstnPerset) * _CTRCLK;
    value_k = 0;
    MotoCtr.PIDAZ.PLast = PstnPerset;

    value = uLimit(value_p + value_i + value_d + value_k, 50);

    value = uLimit(value, 50);

    MotoCtr.PIDAZ.integral = 0;

    MotoCtr.PIDAZ.P = value_p;
    MotoCtr.PIDAZ.I = value_i;
    MotoCtr.PIDAZ.D = value_d;
    MotoCtr.PIDAZ.K = value_k;
    MotoCtr.PIDAZ.Value = value;
    return value;
}

// 计算方位角速度补偿值(前馈)
float GetFeedForwardAZ(void)
{
    float vx, vy, vz, r, p, va;
    const float pi = 3.141592653f;

    vx = Senser.MemsX;
    vy = Senser.MemsY;
    vz = Senser.MemsZ;

    r = Senser.Roll[10] * pi / 180.0f;
    p = Senser.Pitch[10] * pi / 180.0f;

    va = vz * cos(p) * cos(r) - vx * sin(r) + vy * cos(r) * sin(p);

    return -1 * va;
}

float AZSlowStart(float kp, unsigned char time, float error, float maxspeed)
{
    float value_p = 0;
    if (MotoCtr.Timer.az < time) {
        MotoCtr.Timer.az++;
        if (kp != 0) {
            if (fabs(error) > 10 / kp)
                value_p = uLimit(kp * error, maxspeed) * MotoCtr.Timer.az / time;
            else
                value_p = uLimit(kp * error, maxspeed);
        } else
            value_p = 0;
    } else {
        value_p = uLimit(kp * error, maxspeed);
    }

    return value_p;
}

// 计算交叉角速度补偿值(前馈)
extern float GetFeedForwardCR(void);

//交叉缓启动
float RollSlowStart(float kp, unsigned char time, float error, float maxspeed);

//外推滤波
float ExtrapolateFilter(float value, float oldvalue);

// 横滚位置环PID
float PositionControlROLL(float PstnPerset, float* PstnReal)
{
    float kp = 0.0, ki = 0.0, kd = 0.0, td = 0.0, fk = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0;
    float error = 0, error_old = 0;
    float err[2];
    float value = 0;
    int result = 0, i = 0;
    int _lookHeadTime = 0;
    unsigned char time_200ms = 0.2 * _CTRCLK;

    float value_i_old = MotoCtr.PIDROLL.I;
    float value_d_old = MotoCtr.PIDROLL.D;

    kp = P_ROLL;
    ki = I_ROLL / 100.0f;
    td = 0;
    kd = D_ROLL;
    fk = K_ROLL;
    _lookHeadTime = LHT_ROLL;

    //	kp = 40.0;
    //	ki = 0.0;
    //	td = 0.0;
    //	kd = 0.0;
    //	fd = 0.0;
    //	_lookHeadTime = 30;

    //	if(fabs(Senser.MemsX) < 0.5 && fabs(Senser.MemsY) < 0.5 && fabs(Senser.MemsZ) < 0.5)
    //	{
    //		_lookHeadTime = 0;
    //	}

    for (i = 0; i < 300; i++)
        MotoCtr.ROLLP[i] = MotoCtr.ROLLP[i + 1];
    MotoCtr.ROLLP[300] = _180convert(PstnPerset) + 180;

    MotoCtr.PIDROLL.LookHead = Extrapolate(MotoCtr.ROLLP, _lookHeadTime, _CTRCLK, 300) - 180;

    MotoCtr.PIDROLL.LookHeadBeforeL = MotoCtr.PIDROLL.LookHead;
    MotoCtr.PIDROLL.LookHead = ExtrapolateFilter(MotoCtr.PIDROLL.LookHeadBeforeL, PstnPerset);

    error = MotoCtr.PIDROLL.LookHead - PstnReal[10];
    error_old = MotoCtr.PIDROLL.LookHead - PstnReal[9];

    err[1] = _180convert(error);
    err[0] = _180convert(error_old);

    value_p = RollSlowStart(kp, time_200ms, error, 60);

    value_d = td * value_d_old + kd * (err[1] - err[0]);
    result = isinf(value_d);
    if (result != 0)
        value_d = 0;
    //	value_d = 0;

    value_i = uLimit(value_i_old + ki * err[1], 60);
    //	value_i = 0;

    //	value_k = fk * GetFeedForwardCR();
    value_k = fk * (PstnPerset - MotoCtr.PIDROLL.PLast) * _CTRCLK;
    MotoCtr.PIDROLL.PLast = PstnPerset;

    value = uLimit(value_p + value_i + value_d + value_k, 60);
    MotoCtr.PIDROLL.ValueBeforeL = value;

    value = uLimit(value, 60);

    MotoCtr.PIDROLL.P = value_p;
    MotoCtr.PIDROLL.I = value_i;
    MotoCtr.PIDROLL.D = value_d;
    MotoCtr.PIDROLL.K = value_k;
    MotoCtr.PIDROLL.Value = value;

    return value;
}

// 横滚位置环PID，用于指向
float PositionControlROLLPoint(float PstnPerset, float* PstnReal)
{
    float kp = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0, value = 0;
    float error = 0;
    int time_200ms = 0.2 * _CTRCLK;
    int i = 0;

    kp = 5.0;

    for (i = 0; i < 300; i++)
        MotoCtr.ROLLP[i] = MotoCtr.ROLLP[i + 1];
    MotoCtr.ROLLP[300] = PstnPerset + 180;

    error = PstnPerset - PstnReal[10];
    error = _180convert(error);

    value_p = RollSlowStart(kp, time_200ms, error, 30);

    value = uLimit(value_p, 30);

    MotoCtr.PIDROLL.P = value_p;
    MotoCtr.PIDROLL.I = value_i;
    MotoCtr.PIDROLL.D = value_d;
    MotoCtr.PIDROLL.K = value_k;
    MotoCtr.PIDROLL.Value = value;

    return value;
}

// 计算交叉角速度补偿值(前馈)
float GetFeedForwardCR(void)
{
    float vx, vy, vz, a, o, p, r, vc;
    const float pi = 3.141592653f;

    vx = Senser.MemsX;
    vy = Senser.MemsY;
    vz = Senser.MemsZ;

    a = Senser.Angle_AZ[10] * pi / 180.0f;
    o = 30 * pi / 180.0f;
    r = Senser.Roll[10] * pi / 180.0f;
    p = Senser.Pitch[10] * pi / 180.0f;

    vc = vx * (sin(o) * sin(r) - sin(a) * cos(o) * cos(r))
        - vz * (cos(p) * (cos(r) * sin(o) + sin(a) * cos(o) * sin(r)) + cos(a) * cos(o) * sin(p))
        - vy * (sin(p) * (cos(r) * sin(o) + sin(a) * cos(o) * sin(r)) - cos(a) * cos(o) * cos(p));

    return vc;
}

float RollSlowStart(float kp, unsigned char time, float error, float maxspeed)
{
    float value_p = 0;
    if (MotoCtr.Timer.roll < time) {
        MotoCtr.Timer.roll++;
        if (kp != 0) {
            if (fabs(error) > 10 / kp)
                value_p = uLimit(kp * error, maxspeed) * MotoCtr.Timer.roll / time;
            else
                value_p = uLimit(kp * error, maxspeed);
        } else
            value_p = 0;
    } else {
        value_p = uLimit(kp * error, maxspeed);
    }

    return value_p;
}

// 计算俯仰角速度补偿值(前馈)
extern float GetFeedForwardEL(void);

// 俯仰缓启动
float ELSlowStart(float kp, unsigned char time, float error, float maxspeed);

//外推滤波
float ExtrapolateFilter(float value, float oldvalue);

// 俯仰位置环PID, 跟踪
float PositionControlEL(float PstnPerset, float* PstnReal)
{
    float kp = 0.0, ki = 0.0, kd = 0.0, td = 0.0, fk = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0;
    float error = 0, error_old = 0;
    float err[2];
    float FREQ = 500;
    float value = 0;
    int result = 0, i = 0;
    int _lookHeadTime = 0;
    int time_200ms = 0.2 * _CTRCLK;
    float accelerateMax = 0, accelerate = 0;

    float value_i_old = MotoCtr.PIDEL.I;
    float value_d_old = MotoCtr.PIDEL.D;

    kp = P_EL;
    ki = I_EL / 100.0f;
    td = 0;
    kd = D_EL;
    fk = F_EL;
    _lookHeadTime = LHT_EL;

    for (i = 0; i < 300; i++)
        MotoCtr.ELP[i] = MotoCtr.ELP[i + 1];
    MotoCtr.ELP[300] = PstnPerset + 180;

    MotoCtr.PIDEL.LookHead = Extrapolate(MotoCtr.ELP, _lookHeadTime, _CTRCLK, 300) - 180;

    MotoCtr.PIDEL.LookHeadBeforeL = MotoCtr.PIDEL.LookHead;
    MotoCtr.PIDEL.LookHead = ExtrapolateFilter(MotoCtr.PIDEL.LookHeadBeforeL, PstnPerset);

    error = MotoCtr.PIDEL.LookHead - PstnReal[10];
    error_old = MotoCtr.PIDEL.LookHead - PstnReal[9];

    err[1] = _180convert(error);
    err[0] = _180convert(error_old);

    value_p = ELSlowStart(kp, time_200ms, error, 80);

    value_d = td * value_d_old + kd * (err[1] - err[0]);
    result = isinf(value_d);
    if (result != 0)
        value_d = 0;
    value_d = 0;

    if (ki == 0) {
        value_i_old = 0;
    }
    value_i = uLimit(value_i_old + ki * err[1], 80);

    //	value_k = fk * GetFeedForwardEL();
    value_k = fk * (PstnPerset - MotoCtr.PIDEL.PLast) * _CTRCLK;
    MotoCtr.PIDEL.PLast = PstnPerset;

    value = uLimit(value_p + value_i + value_d + value_k, 80);
    MotoCtr.PIDEL.ValueBeforeL = value;

    value = uLimit(value, 80);

    MotoCtr.PIDEL.P = value_p;
    MotoCtr.PIDEL.I = value_i;
    MotoCtr.PIDEL.D = value_d;
    MotoCtr.PIDEL.K = value_k;
    MotoCtr.PIDEL.Value = value;

    return value;
}

// 俯仰位置环PID，用于指向
float PositionControlELPoint(float PstnPerset, float* PstnReal)
{
    float kp = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0, value = 0;
    float error = 0;
    int time_200ms = 0.2 * _CTRCLK;
    int i = 0;

    kp = 5.0;

    for (i = 0; i < 300; i++)
        MotoCtr.ELP[i] = MotoCtr.ELP[i + 1];
    MotoCtr.ELP[300] = PstnPerset + 180;

    error = PstnPerset - PstnReal[10];
    error = _180convert(error);

    value_p = ELSlowStart(kp, time_200ms, error, 30);

    MotoCtr.PIDEL.PLast = PstnPerset;

    value = uLimit(value_p, 30);

    MotoCtr.PIDEL.P = value_p;
    MotoCtr.PIDEL.I = value_i;
    MotoCtr.PIDEL.D = value_d;
    MotoCtr.PIDEL.K = value_k;
    MotoCtr.PIDEL.Value = value;

    return value;
}

// 计算俯仰角速度补偿值(前馈)
float GetFeedForwardEL(void)
{
    float vx, vy, vz, a, o, c, p, r, ve;
    const float pi = 3.141592653;

    vx = Senser.MemsX;
    vy = Senser.MemsY;
    vz = Senser.MemsZ;

    a = Senser.Angle_AZ[10] * pi / 180.0f;
    c = Senser.Angle_ROLL[10] * pi / 180.0f;
    o = 30 * pi / 180.0f;
    r = Senser.Roll[10] * pi / 180.0f;
    p = Senser.Pitch[10] * pi / 180.0f;

    ve = vx * (cos(r) * (cos(a) * cos(c) - sin(a) * sin(c) * sin(o)) - sin(c) * cos(o) * sin(r))
        + vy * (sin(p) * (sin(r) * (cos(a) * cos(c) - sin(a) * sin(c) * sin(o)) + sin(c) * cos(o) * cos(r)) + cos(p) * (cos(c) * sin(a) + cos(a) * sin(c) * sin(o)))
        + vz * (cos(p) * (sin(r) * (cos(a) * cos(c) - sin(a) * sin(c) * sin(o)) + sin(c) * cos(o) * cos(r)) - sin(p) * (cos(c) * sin(a) + cos(a) * sin(c) * sin(o)));

    return ve;
}

float ELSlowStart(float kp, unsigned char time, float error, float maxspeed)
{
    float value_p = 0;
    if (MotoCtr.Timer.el < time) {
        MotoCtr.Timer.el++;
        if (kp != 0) {
            if (fabs(error) > 10 / kp)
                value_p = uLimit(kp * error, maxspeed) * MotoCtr.Timer.el / time;
            else
                value_p = uLimit(kp * error, maxspeed);
        } else
            value_p = 0;
    } else {
        value_p = uLimit(kp * error, maxspeed);
    }

    return value_p;
}

// 极化位置环PID
float PositionControlPOL(float PstnPerset, float* PstnReal)
{
    float kp = 0.0, ki = 0.0, kd = 0.0, td = 0.0, fd = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0;
    float error = 0, error_old = 0;
    float err[2];
    float value = 0;
    int result = 0, i = 0;
    int _lookHeadTime = 0;

    float value_i_old = MotoCtr.PIDROLL.I;
    float value_d_old = MotoCtr.PIDROLL.D;

    kp = P_POL;
    ki = I_POL / 100.0f;
    td = 0;
    kd = D_POL;
    fd = K_POL;
    _lookHeadTime = LHT_POL;

    for (i = 0; i < 300; i++)
        MotoCtr.POLP[i] = MotoCtr.POLP[i + 1];
    MotoCtr.POLP[300] = PstnPerset + 180;

    MotoCtr.PIDPOL.LookHead = Extrapolate(MotoCtr.POLP, _lookHeadTime, _CTRCLK, 300) - 180;

    MotoCtr.PIDPOL.LookHeadBeforeL = MotoCtr.PIDPOL.LookHead;

    error = MotoCtr.PIDPOL.LookHead - PstnReal[10];
    error_old = MotoCtr.PIDPOL.LookHead - PstnReal[9];

    //	err[1] = _180convert(error);
    //	err[0] = _180convert(error_old);
    err[1] = error;
    err[0] = error_old;

    value_p = uLimit(kp * error, 45);

    value_d = td * value_d_old + kd * (err[1] - err[0]);
    result = isinf(value_d);
    if (result != 0)
        value_d = 0;

    value_i = uLimit(value_i_old + ki * err[1], 45);

    value_k = fd * (1 * Senser.MemsX * sin(Senser.Angle_AZ[10] * PI / 180.0f) + -1 * Senser.MemsY * cos(Senser.Angle_AZ[10] * PI / 180.0f));

    value = uLimit(value_p + value_i + value_d + value_k, 45);
    MotoCtr.PIDPOL.ValueBeforeL = value;

    value = uLimit(value, 45);

    MotoCtr.PIDPOL.P = value_p;
    MotoCtr.PIDPOL.I = value_i;
    MotoCtr.PIDPOL.D = value_d;
    MotoCtr.PIDPOL.K = value_k;
    MotoCtr.PIDPOL.Value = value;

    return -1 * value;
}

// 极化位置环PID，用于指向
float PositionControlPOLPoint(float PstnPerset, float* PstnReal)
{
    float kp = 0.0;
    float value_p = 0, value_i = 0, value_d = 0, value_k = 0, value = 0;
    float error = 0;
    int time_200ms = 0.2 * _CTRCLK;
    int i = 0;

    kp = 5.0;

    for (i = 0; i < 300; i++)
        MotoCtr.POLP[i] = MotoCtr.POLP[i + 1];
    MotoCtr.POLP[300] = PstnPerset + 180;

    error = PstnPerset - PstnReal[10];
    error = _180convert(error);

    if (MotoCtr.Timer.pol < time_200ms) {
        MotoCtr.Timer.pol++;
        if (kp != 0) {
            if (fabs(error) > 10 / kp)
                value_p = uLimit(kp * error, 30) * MotoCtr.Timer.pol / time_200ms;
            else
                value_p = uLimit(kp * error, 30);
        } else
            value_p = 0;
    } else {
        value_p = uLimit(kp * error, 30);
    }

    value = uLimit(value_p, 30);

    MotoCtr.PIDPOL.P = value_p;
    MotoCtr.PIDPOL.I = value_i;
    MotoCtr.PIDPOL.D = value_d;
    MotoCtr.PIDPOL.K = value_k;
    MotoCtr.PIDPOL.Value = value;

    return value;
}
