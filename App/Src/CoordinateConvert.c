#include "CoordinateConvert.h"
#include "MotorControl.h"
#include "ParametersDefine.h"
#include "Senser.h"
#include "math.h"
#include "stdlib.h"

// 计算得到目标卫星的空间指向角
void GetSatelliteAngle(float* AZ, float* EL, float* Pol)
{
    float antenna_longitude = 0;
    float antenna_latitude = 0;
    float satellite_longitude = 0;
    float temp = 0;

    antenna_longitude = Senser.GPSX * PI / 180;
    antenna_latitude = Senser.GPSY * PI / 180;
    satellite_longitude = SatellitesInfo.longitude * PI / 180;

    *AZ = (PI - atan(tan(-antenna_longitude + satellite_longitude) / sin(antenna_latitude))) * 180 / PI;

    temp = cos(-antenna_longitude + satellite_longitude) * cos(antenna_latitude);

    *EL = (atan((temp - 0.15126f) / (sqrt(1 - temp * temp)))) * 180 / PI;

    *Pol = (atan(sin(-antenna_longitude + satellite_longitude) / tan(antenna_latitude))) * 180 / PI;
}

// 计算得到参考卫星对于天线的空间指向角及极化角
void GetAngleGOfNormalSate(float* AZ, float* EL, float* Pol)
{
    float antenna_longitude = 0;
    float antenna_latitude = 0;
    float satellite_longitude = 0;
    float temp = 0;

    antenna_longitude = Senser.GPSX * PI / 180.0f;
    antenna_latitude = Senser.GPSY * PI / 180.0f;
    satellite_longitude = SatellitesInfo.longitude * PI / 180.0f;

    *AZ = (PI - atan(tan(-antenna_longitude + satellite_longitude) / sin(antenna_latitude))) * 180 / PI;

    temp = cos(-antenna_longitude + satellite_longitude) * cos(antenna_latitude);

    *EL = (atan((temp - 0.15126f) / (sqrt(1 - temp * temp)))) * 180 / PI;

    *Pol = (atan(sin(-antenna_longitude + satellite_longitude) / tan(antenna_latitude))) * 180 / PI;
}

//三轴天线空间坐标系到天线坐标系角度计算
//注：交叉安装轴与方位转盘承120°
//惯导安装于天线底座，不跟随方位转动
void GtoB(float azg, float elg, float head, float pitch, float roll, float* A, float* E, float* C)
{
    float eTem = 0.0;
    float azgRad, elgRad, headRad, pitchRad, rollRad, azbRad, cRad;
    float a, e, c;
    const float pi = 3.141592653f;

    float t = CROSS_ERR;
    azgRad = azg * pi / 180.0f;
    elgRad = elg * pi / 180.0f;
    headRad = head * pi / 180.0f;
    pitchRad = pitch * pi / 180.0f;
    rollRad = roll * pi / 180.0f;

    a = GetA(azgRad, elgRad, headRad, pitchRad, rollRad);

    azbRad = a * pi / 180.0f;
    c = GetC(azbRad, pitchRad, rollRad);

    cRad = c * pi / 180.0f;
    e = GetE(elgRad, azbRad, cRad, pitchRad, rollRad);

    //经过实际测试，在有姿态角情况下需要对俯仰角进行补偿

    eTem = e * ELBUCHANGA + ELBUCHANGB;
    if (LINEARFLAG == 1) {
        if ((eTem - e) >= THRESHOLDUP) {
            e += VALUEUP;
        } else if ((eTem - e) <= THRESHOLDDOWN) {
            e += VALUEDOWN;
        } else {
            e = eTem;
        }
    } else if (LINEARFLAG == 2) {
        if ((e - (elg - 30)) < THRESHOLDDOWN) {
            e = e - (e - (elg - 30)) * VALUEDOWN;
        } else if ((e - (elg - 30)) > THRESHOLDUP) {
            e = e - (e - (elg - 30)) * VALUEUP;
        } else {
            e = e;
        }
    } else {
        e = eTem;
    }
    *A = a;
    *E = e;
    *C = c;
}

//计算得到方位转角，输入参数为弧度制
float GetA(float agRad, float egRad, float headRad, float pitchRad, float rollRad)
{
    float xg, yg, h, p, r, o, t;
    float m, n, c, B, sita, temp, ab;
    const float pi = 3.141592653;

    o = OFFSETO * pi / 180.0f;
    t = CROSS_ERR * pi / 180.0f;

    xg = cos(egRad) * sin(agRad);
    yg = cos(egRad) * cos(agRad);

    h = headRad;
    p = pitchRad;
    r = rollRad;

    m = xg * (cos(h) * cos(p) * sin(t) + cos(o) * cos(t) * (cos(r) * sin(h) - cos(h) * sin(p) * sin(r)))
        - yg * (cos(p) * sin(h) * sin(t) - cos(o) * cos(t) * (cos(h) * cos(r) + sin(h) * sin(p) * sin(r)));

    n = xg * (sin(t) * (cos(r) * sin(h) - cos(h) * sin(p) * sin(r)) - cos(h) * cos(o) * cos(p) * cos(t))
        + yg * (sin(t) * (cos(h) * cos(r) + sin(h) * sin(p) * sin(r)) + cos(o) * cos(p) * cos(t) * sin(h));

    c = xg * cos(t) * sin(o) * (sin(h) * sin(r) + cos(h) * cos(r) * sin(p))
        + yg * cos(t) * sin(o) * (cos(h) * sin(r) - sin(h) * cos(r) * sin(p));

    B = atan(m / n);
    B = B * 180.0f / pi;

    temp = c / sqrt(m * m + n * n);
    sita = asin(temp);
    sita = sita * 180.0f / pi;

    ab = B - sita;
    if (n >= 0) {
        ab = ab + 270;
    } else {
        ab = ab + 90;
    }

    if (ab >= 360) {
        ab -= 360;
    }
    if (ab < 0) {
        ab += 360;
    }

    return ab;
}

//计算得到方位转角，输入参数为角度制
float GetA_(float ag, float eg, float head, float pitch, float roll)
{
    float xg, yg, h, p, r, o, agRad, egRad;
    float m, n, c, B, sita, temp, ab;
    const float pi = 3.141592653;

    o = OFFSETO * pi / 180.0f;

    agRad = ag * pi / 180.0f;
    egRad = eg * pi / 180.0f;

    xg = cos(egRad) * sin(agRad);
    yg = cos(egRad) * cos(agRad);

    h = head * pi / 180.0f;
    p = pitch * pi / 180.0f;
    r = roll * pi / 180.0f;

    m = xg * cos(o) * (sin(h) * cos(r) - cos(h) * sin(p) * sin(r))
        + yg * cos(o) * (cos(h) * cos(r) + sin(h) * sin(p) * sin(r));

    n = -xg * cos(h) * cos(o) * cos(p)
        + yg * sin(h) * cos(o) * cos(p);

    c = xg * sin(o) * (sin(h) * sin(r) + cos(h) * cos(r) * sin(p))
        + yg * sin(o) * (cos(h) * sin(r) - sin(h) * cos(r) * sin(p));

    B = atan(m / n);
    B = B * 180.0f / pi;

    temp = c / sqrt(m * m + n * n);
    sita = asin(temp);
    sita = sita * 180.0f / pi;

    ab = B - sita;
    if (n >= 0) {
        ab = ab + 270;
    } else {
        ab = ab + 90;
    }

    if (ab >= 360) {
        ab -= 360;
    }
    if (ab < 0) {
        ab += 360;
    }

    return ab;
}

//计算得到交叉转角
float GetC(float azbRad, float pitchRad, float rollRad)
{
    float a, p, r, o, tanC, c, t;
    const float pi = 3.141592653f;

    o = OFFSETO * pi / 180.0f;
    t = CROSS_ERR * pi / 180.0f;

    a = azbRad;

    p = pitchRad;
    r = rollRad;

    tanC = (-(sin(t) * (cos(o) * (cos(a) * sin(p) - cos(p) * sin(a) * sin(r)) + cos(p) * cos(r) * sin(o))) + cos(t) * (sin(a) * sin(p) + cos(a) * cos(p) * sin(r)))
        / (sin(o) * (cos(a) * sin(p) - cos(p) * sin(a) * sin(r)) - cos(o) * cos(p) * cos(r));

    c = atan(tanC);
    c = c * 180.0f / pi;

    return c;
}

//计算得到俯仰转角
float GetE(float egRad, float abRad, float cRad, float pitchRad, float rollRad)
{
    float zg, a, c, p, r, o, e, m, n, temp, sita, B, t;
    const float pi = 3.141592653;

    zg = sin(egRad);

    o = OFFSETO * pi / 180.0f;
    t = CROSS_ERR * pi / 180.0f;

    a = abRad;
    c = cRad;
    p = pitchRad;
    r = rollRad;

    n = -(cos(c) * (sin(o) * (cos(a) * sin(p) - sin(a) * cos(p) * sin(r)) - cos(o) * cos(p) * cos(r)))
        - sin(c) * (sin(t) * (cos(o) * (cos(a) * sin(p) - cos(p) * sin(a) * sin(r)) + cos(p) * cos(r) * sin(o)) + cos(t) * (sin(a) * sin(p) + cos(a) * cos(p) * sin(r)));

    m = cos(t) * (cos(o) * (cos(a) * sin(p) - sin(a) * cos(p) * sin(r)) + cos(p) * cos(r) * sin(o))
        - sin(t) * (sin(a) * sin(p) + cos(a) * cos(p) * sin(r));

    temp = zg / sqrt(m * m + n * n);
    sita = asin(temp);
    sita = sita * 180.0f / pi;

    B = atan(m / n);
    B = B * 180.0f / pi;

    e = sita - B;
    return e;
}

//计算得到空间指向
void BtoG(float azb, float elb, float crb, float head, float pitch, float roll, float* azg, float* elg)
{
    float a, e, c, h, p, r, o, xg, yg, zg, ag, eg;
    const float pi = 3.141592653;

    a = azb * pi / 180.0f;
    e = elb * pi / 180.0f;
    c = crb * pi / 180.0f;

    h = head * pi / 180.0f;
    p = pitch * pi / 180.0f;
    r = roll * pi / 180.0f;

    o = OFFSETO * pi / 180.0f;

    xg = cos(o) * (sin(a) * (cos(h) * cos(r) + sin(h) * sin(p) * sin(r)) + cos(a) * sin(h) * cos(p)) + sin(o) * (cos(h) * sin(r) - sin(h) * cos(r) * sin(p));
    yg = -(cos(o) * (sin(a) * (sin(h) * cos(r) - cos(h) * sin(p) * sin(r)) - cos(a) * cos(h) * cos(p))) - sin(o) * (sin(h) * sin(r) + cos(h) * cos(r) * sin(p));

    ag = atan(xg / yg);
    ag = ag * 180.0f / pi;
    if (xg > 0) {
        if (yg > 0) {
            ag = ag;
        } else {
            ag = ag + 180;
        }
    } else {
        if (yg > 0) {
            ag = ag + 360;
        } else {
            ag = ag + 180;
        }
    }

    zg = cos(e) * (cos(o) * (cos(a) * sin(p) - sin(a) * cos(p) * sin(r)) + cos(p) * cos(r) * sin(o))
        - sin(e) * (cos(c) * (sin(o) * (cos(a) * sin(p) - sin(a) * cos(p) * sin(r)) - cos(o) * cos(p) * cos(r)) + sin(c) * (sin(a) * sin(p) + cos(a) * cos(p) * sin(r)));
    eg = asin(zg);
    eg = eg * 180.0f / pi;

    *azg = ag;
    *elg = eg;
}

//计算得到当前航向角
float GetH(float AZg, float ELg, float AZb, float Pitch, float Roll)
{
    float agRad, egRad;
    float xg, yg, a, p, r, h, o, B, m, n;
    const float pi = 3.141592653;

    agRad = AZg * pi / 180.0f;
    egRad = ELg * pi / 180.0f;

    a = AZb * pi / 180.0f;
    p = Pitch * pi / 180.0f;
    r = Roll * pi / 180.0f;

    o = OFFSETO * pi / 180.0f;

    xg = cos(egRad) * sin(agRad);
    yg = cos(egRad) * cos(agRad);

    m = yg * (sin(o) * sin(r) + sin(a) * cos(o) * cos(r)) - xg * (cos(a) * cos(o) * cos(p) - cos(r) * sin(o) * sin(p) + sin(a) * cos(o) * sin(p) * sin(r));
    n = xg * (sin(o) * sin(r) + sin(a) * cos(o) * cos(r)) + yg * (cos(a) * cos(o) * cos(p) - cos(r) * sin(o) * sin(p) + sin(a) * cos(o) * sin(p) * sin(r));

    B = atan(m / n);
    B = B * 180.0f / pi;

    if (m > 0) {
        if (n > 0) {
            h = -1 * B + 360;
        } else {
            h = -1 * B + 180;
        }
    } else {
        if (n > 0) {
            h = -1 * B;
        } else {
            h = -1 * B + 180;
        }
    }

    return h;
}

/// <summary>
/// 计算天线和陀螺的实际角度？？？？
///	函数翻译应为"矩阵乘法"
/// </summary>
/// <param name="a">?</param>
/// <param name="b">?</param>
/// <param name="c">?</param>
/// <param name="l">a数组长度</param>
/// <param name="m">b数组长度</param>
/// <param name="n">c数组长度</param>
void matrix_multi(float* a, float* b, float* c, int l, int m, int n)
{
    int i = 0, j = 0, k = 0;
    for (i = 0; i < l; i = i + 1) {
        for (j = 0; j < n; j = j + 1) {
            c[i * n + j] = 0;
            for (k = 0; k < m; k = k + 1) {
                c[i * n + j] += a[i * m + k] * b[k * n + j];
            }
        }
    }
}

/// <summary>
/// 坐标系转换，地理坐标系->载体坐标系
/// </summary>
/// <param name="AZg">方位角，地理坐标系</param>
/// <param name="ELg">俯仰角，地理坐标系</param>
/// <param name="Head">载体姿态，航向角</param>
/// <param name="Pitch">载体姿态，纵摇角</param>
/// <param name="Roll">载体姿态，横滚角</param>
/// <param name="Roll">当前方位角</param>
/// <param name="AZb">方位角，天线坐标系</param>
/// <param name="ELb">俯仰角，天线坐标系</param>
void NewConvertGTOB(float AZg, float ELg, float Head, float Pitch, float Roll, float azbNow, float* AZb, float* ELb)
{
    //地理坐标系下方位、俯仰角，弧度制
    float az0 = 0, el0 = 0;
    //载体姿态角，弧度制
    float hRad = 0, pRad = 0, rRad = 0;
    //载体坐标系下方位、俯仰角，弧度制
    float az1 = 0, el1 = 0;

    //g系到b系角度矩阵
    float x1 = 0, y1 = 0;
    //float z1 = 0;
    //float x2 = 0, y2 = 0, z2 = 0;

    az0 = AZg * PI / 180.0f;
    el0 = ELg * PI / 180.0f;
    //pol0 = Polg * PI / 180;

    hRad = Head * PI / 180.0f;
    pRad = Pitch * PI / 180.0f;
    rRad = Roll * PI / 180.0f;

    x1 = cos(el0) * (cos(rRad) * sin(az0 - hRad) + sin(rRad) * sin(pRad) * cos(az0 - hRad))
        - sin(rRad) * cos(pRad) * sin(el0) + 0.0001;
    y1 = cos(pRad) * cos(el0) * cos(az0 - hRad) + sin(pRad) * sin(el0) + 0.0001;

    az1 = atan(x1 / y1) * 180.0f / PI;
    if (y1 < 0) {
        if (x1 > 0)
            az1 = 180 + az1;
        else
            az1 = 180 + az1;
    } else {
        if (x1 < 0)
            az1 = 360 + az1;
    }

    el1 = asin(cos(el0) * (sin(rRad) * sin(az0 - hRad) - cos(rRad) * sin(pRad) * cos(az0 - hRad))
        + cos(rRad) * cos(pRad) * sin(el0));
    el1 = el1 * 180.0f / PI;

    if (az1 >= 360) {
        az1 -= 360;
    }
    if (az1 < 0) {
        az1 += 360;
    }

    if (AZb != NULL) {
        *AZb = az1;
    }
    if (ELb != NULL) {
        *ELb = el1;
    }
}

/// <summary>
/// 获取极化角度，载体坐标系
/// </summary>
/// <param name="AZb">方位角，载体坐标系</param>
/// <param name="ELb">俯仰角，载体坐标系</param>
/// <param name="Head">载体姿态，航向角</param>
/// <param name="Pitch">载体姿态，纵摇角</param>
/// <param name="Roll">载体姿态，横滚角</param>
/// <param name="Pol">极化角，载体坐标系</param>
void GetPol(float AZb, float ELb, float Head, float Pitch, float Roll, float* Pol)
{
    //天线方位、俯仰角、极化角，载体坐标系
    float azRad = 0, elRad = 0, polRad = 0;

    //载体姿态角，弧度制
    float hRad = 0, pRad = 0, rRad = 0;

    float tempPol = 0;

    //计算极化角度使用的变量，暂未完全明白
    float d[3][3], e[3][3], f[3][3], g[3][3], hh[3][1], n[3][1], a[3][3], b[3][3], c[3][3];
    float seaa = 0, seap = 0;
    float EAh1 = 0, EAh2 = 0;

    float antenna_longitude = 0; //载体经度,rad
    float antenna_latitude = 0; //载体纬度,rad
    float satellite_longitude = 0; //卫星经度,rad

    int i = 0, j = 0, k = 0;

    azRad = AZb * PI / 180;
    elRad = ELb * PI / 180;

    hRad = Head * PI / 180;
    pRad = Pitch * PI / 180;
    rRad = Roll * PI / 180;

    antenna_longitude = Senser.GPSX * PI / 180;
    antenna_latitude = Senser.GPSY * PI / 180;
    satellite_longitude = SatellitesInfo.longitude * PI / 180;

    a[0][0] = 0;
    a[0][1] = 0;
    a[0][2] = 0;

    a[1][0] = 0;
    a[1][1] = 0;
    a[1][2] = 0;

    a[2][0] = 0;
    a[2][1] = 0;
    a[2][2] = 0;

    b[0][0] = 0;
    b[0][1] = 0;
    b[0][2] = 0;

    b[1][0] = 0;
    b[1][1] = 0;
    b[1][2] = 0;

    b[2][0] = 0;
    b[2][1] = 0;
    b[2][2] = 0;

    c[0][0] = 0;
    c[0][1] = 0;
    c[0][2] = 0;

    c[1][0] = 0;
    c[1][1] = 0;
    c[1][2] = 0;

    c[2][0] = 0;
    c[2][1] = 0;
    c[2][2] = 0;

    n[0][0] = 0;

    //----------------极化计算start-------------------
    seaa = atan(sin(antenna_longitude - satellite_longitude) / tan(antenna_latitude));
    seap = atan(sqrt(1 - cos(antenna_longitude - satellite_longitude) * cos(antenna_latitude) * cos(antenna_longitude - satellite_longitude) * cos(antenna_latitude))
        / (6.6108498 - cos(antenna_longitude - satellite_longitude) * cos(antenna_latitude)));
    MotoCtr.PolgPerset = seaa * 180 / PI;

    d[0][0] = -cos(azRad) * cos(elRad);
    d[0][1] = sin(azRad) * cos(elRad);
    d[0][2] = sin(elRad);

    d[1][0] = -sin(azRad);
    d[1][1] = -cos(azRad);
    d[1][2] = 0;

    d[2][0] = cos(azRad) * sin(elRad);
    d[2][1] = -sin(azRad) * sin(elRad);
    d[2][2] = cos(elRad);

    e[0][0] = cos(pRad) * cos(hRad);
    e[0][1] = -cos(pRad) * sin(hRad);
    e[0][2] = -sin(pRad);

    e[1][0] = -sin(pRad) * sin(rRad) * cos(hRad) + cos(rRad) * sin(hRad);
    e[1][1] = cos(rRad) * cos(hRad) + sin(pRad) * sin(rRad) * sin(hRad);
    e[1][2] = -sin(rRad) * cos(pRad);

    e[2][0] = sin(rRad) * sin(hRad) + sin(pRad) * cos(rRad) * cos(hRad);
    e[2][1] = -sin(pRad) * cos(rRad) * sin(hRad) + sin(rRad) * cos(hRad);
    e[2][2] = cos(pRad) * cos(rRad);

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
                a[i][j] += d[i][k] * e[k][j];
            }
        }
    }

    f[0][0] = sin(antenna_latitude) * cos(antenna_longitude - satellite_longitude);
    f[0][1] = sin(antenna_latitude) * sin(antenna_longitude - satellite_longitude);
    f[0][2] = -cos(antenna_latitude);

    f[1][0] = -sin(antenna_longitude - satellite_longitude);
    f[1][1] = cos(antenna_longitude - satellite_longitude);
    f[1][2] = 0;

    f[2][0] = cos(antenna_latitude) * cos(antenna_longitude - satellite_longitude);
    f[2][1] = cos(antenna_latitude) * sin(antenna_longitude - satellite_longitude);
    f[2][2] = sin(antenna_latitude);

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
                b[i][j] += a[i][k] * f[k][j];
            }
        }
    }

    g[0][0] = 0;
    g[0][1] = 0;
    g[0][2] = -1;

    g[1][0] = 0;
    g[1][1] = 1;
    g[1][2] = 0;

    g[2][0] = 1;
    g[2][1] = 0;
    g[2][2] = 0;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            for (k = 0; k < 3; k++) {
                c[i][j] += b[i][k] * g[k][j];
            }
        }
    }

    hh[0][0] = (sin(seap) * sin(seap) * sin(seaa) * cos(seaa)) / (sqrt(1 - sin(seaa) * sin(seap) * sin(seaa) * sin(seap)));
    hh[1][0] = sqrt(1 - sin(seaa) * sin(seap) * sin(seaa) * sin(seap));
    hh[2][0] = sin(seap) * cos(seap) * sin(seaa) / (sqrt(1 - sin(seaa) * sin(seap) * sin(seaa) * sin(seap)));

    matrix_multi(c[0], hh[0], n[0], 3, 3, 1);

    //EAh0 = n[0][0];
    EAh1 = n[1][0];
    EAh2 = n[2][0];
    polRad = -1 * atan(EAh2 / EAh1);

    tempPol = polRad * 180 / PI;

    if (tempPol < 0)
        tempPol += 360;
    if (tempPol > 360)
        tempPol += 360;
    *Pol = tempPol;
    //----------------极化计算end---------------------
}

/// <summary>
/// 坐标系转换，载体坐标系->地理坐标系
/// </summary>
/// <param name="AZb">方位角，载体坐标系</param>
/// <param name="ELb">俯仰角，载体坐标系</param>
/// <param name="Head">载体姿态，航向角</param>
/// <param name="Pitch">载体姿态，纵摇角</param>
/// <param name="Roll">载体姿态，横滚角</param>
/// <param name="AZg">方位角，地理坐标系</param>
/// <param name="ELg">俯仰角，地理坐标系</param>
void NewConvertBTOG(float AZb, float ELb, float Head, float Pitch, float Roll, float* AZg, float* ELg)
{
    float hRad = 0, pRad = 0, rRad = 0, azb = 0, elb = 0;
    float x1 = 0, y1 = 0;
    float tempAZg = 0, tempELg = 0;

    hRad = Head * PI / 180.0f;
    pRad = Pitch * PI / 180.0f;
    rRad = Roll * PI / 180.0f;
    azb = AZb * PI / 180.0f;
    elb = ELb * PI / 180.0f;

    x1 = (cos(hRad) * cos(rRad) + sin(hRad) * sin(pRad) * sin(rRad)) * cos(elb) * sin(azb) + sin(hRad) * cos(pRad) * cos(elb) * cos(azb) + (cos(hRad) * sin(rRad) - sin(hRad) * sin(pRad) * cos(rRad)) * sin(elb) + 0.00001;
    y1 = (-1 * sin(hRad) * cos(rRad) + cos(hRad) * sin(pRad) * sin(rRad)) * cos(elb) * sin(azb) + cos(hRad) * cos(pRad) * cos(elb) * cos(azb) + (-1 * sin(hRad) * sin(rRad) - cos(hRad) * sin(pRad) * cos(rRad)) * sin(elb) + 0.00001;

    tempAZg = atan(x1 / y1);
    tempAZg = tempAZg / PI * 180.0f;
    if (x1 > 0 && y1 < 0)
        tempAZg = tempAZg + 180.0f;
    if (x1 < 0 && y1 < 0)
        tempAZg = tempAZg + 180.0f;
    if (x1 < 0 && y1 > 0)
        tempAZg = tempAZg + 360.0f;

    tempELg = asin(-cos(pRad) * sin(rRad) * cos(elb) * sin(azb)
        + sin(pRad) * cos(elb) * cos(azb)
        + cos(pRad) * cos(rRad) * sin(elb));
    tempELg = tempELg / PI * 180.0f;
    *AZg = tempAZg;
    *ELg = tempELg;
}
