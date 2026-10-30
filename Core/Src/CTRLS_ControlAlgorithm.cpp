// Created by admin on 5/10/2026.

#include "CTRLS_ControlAlgorithm.h"
#include <cstdlib>  // for abs

namespace {

/// Motor thrust curve sample times (s). Pairs with kThrustN.
static constexpr float kThrustT[] = {
    0.07f, 0.211f, 0.353f, 0.495f, 0.636f, 0.778f, 0.92f, 1.061f,
    1.203f, 1.345f, 1.486f, 1.628f, 1.771f, 1.914f, 2.055f, 2.197f,
    2.339f, 2.48f, 2.622f, 2.764f, 2.905f, 3.047f, 3.189f, 3.33f,
    3.473f, 3.616f
};

/// Thrust (N) at each time in kThrustT.
static constexpr float kThrustN[] = {
    2441.945f, 2495.46f, 2556.133f, 2601.596f, 2637.66f, 2660.804f, 2676.486f, 2687.081f,
    2695.807f, 2694.493f, 2684.268f, 2667.289f, 2629.961f, 2578.923f, 2522.074f, 2461.704f,
    2393.518f, 2303.939f, 2201.61f, 2097.461f, 2010.409f, 1275.776f, 418.836f, 17.586f,
    3.669f, 0.0f
};

/// Mach numbers for the drag table, 0.01 to 1.0. Pairs with kDragCd.
static constexpr float kDragMach[] = {
    0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f,
    0.09f, 0.1f, 0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f,
    0.17f, 0.18f, 0.19f, 0.2f, 0.21f, 0.22f, 0.23f, 0.24f,
    0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.3f, 0.31f, 0.32f,
    0.33f, 0.34f, 0.35f, 0.36f, 0.37f, 0.38f, 0.39f, 0.4f,
    0.41f, 0.42f, 0.43f, 0.44f, 0.45f, 0.46f, 0.47f, 0.48f,
    0.49f, 0.5f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f, 0.56f,
    0.57f, 0.58f, 0.59f, 0.6f, 0.61f, 0.62f, 0.63f, 0.64f,
    0.65f, 0.66f, 0.67f, 0.68f, 0.69f, 0.7f, 0.71f, 0.72f,
    0.73f, 0.74f, 0.75f, 0.76f, 0.77f, 0.78f, 0.79f, 0.8f,
    0.81f, 0.82f, 0.83f, 0.84f, 0.85f, 0.86f, 0.87f, 0.88f,
    0.89f, 0.9f, 0.91f, 0.92f, 0.93f, 0.94f, 0.95f, 0.96f,
    0.97f, 0.98f, 0.99f, 1.0f
};

/// Drag coefficient at each Mach number in kDragMach, loaded as both the power-on and the power-off drag.
static constexpr float kDragCd[] = {
    0.569051589f, 0.52605377f, 0.525832698f, 0.525615889f, 0.5254033f, 0.52519502f, 0.524991009f, 0.524791277f,
    0.524595889f, 0.524404882f, 0.524218202f, 0.524035931f, 0.523858126f, 0.523684782f, 0.523515978f, 0.52335172f,
    0.523192083f, 0.523037128f, 0.522886922f, 0.522741522f, 0.522600976f, 0.522465341f, 0.522334793f, 0.522209327f,
    0.522089082f, 0.522456875f, 0.522829131f, 0.523205929f, 0.523587459f, 0.523973811f, 0.524365069f, 0.524761467f,
    0.525163147f, 0.525570251f, 0.525982943f, 0.526401418f, 0.526825879f, 0.527256541f, 0.527693567f, 0.528137297f,
    0.528587922f, 0.529045713f, 0.529510946f, 0.529983978f, 0.530465078f, 0.530954593f, 0.531452951f, 0.531960482f,
    0.532477693f, 0.533004953f, 0.533542788f, 0.534091723f, 0.534652347f, 0.535225235f, 0.535811092f, 0.536410567f,
    0.53702454f, 0.537653713f, 0.538299096f, 0.538961663f, 0.540487449f, 0.542000834f, 0.543533427f, 0.545086685f,
    0.546662174f, 0.548261658f, 0.54988696f, 0.551540284f, 0.553223938f, 0.554940606f, 0.555828201f, 0.556660936f,
    0.557492007f, 0.55832138f, 0.559149071f, 0.559975082f, 0.560799396f, 0.561622012f, 0.562443031f, 0.563262365f,
    0.566191044f, 0.569118026f, 0.572043456f, 0.574967195f, 0.577889305f, 0.580809862f, 0.583728762f, 0.586646034f,
    0.589561727f, 0.59247585f, 0.596504685f, 0.608591193f, 0.638876354f, 0.681316916f, 0.723757477f, 0.766198039f,
    0.808638601f, 0.851079162f, 0.893519724f, 0.935960286f
};

/// One row of kWeatherProfile.
struct WeatherPoint {
    float altitude_agl_m;   ///< Height above the launch site (m).
    float rho;              ///< Air density (kg/m³).
    float temperature_K;    ///< Air temperature (K).
    float g;                ///< Gravity (m/s²).
    float wind_x;           ///< Wind, world frame (m/s).
    float wind_y;           ///< Wind, world frame (m/s).
};

/**
 * @brief Atmosphere and wind for Physics::weatherAtAltitude(), in 50 m steps.
 *
 * Generated from FV-Controls/rockets/Control Freak/data/weather/weather_profile.csv; capped at 4000 m AGL.
 */
static constexpr WeatherPoint kWeatherProfile[] = {
    {0.000f, 1.02020119f, 310.877052f, 9.79146002f, 4.37660828f, 2.2574781f},
    {50.000f, 1.01559781f, 310.5356f, 9.79130576f, 4.39296528f, 2.26361198f},
    {100.000f, 1.01099443f, 310.194147f, 9.79115149f, 4.40932229f, 2.26974586f},
    {150.000f, 1.00701626f, 309.721649f, 9.79099723f, 4.489026f, 2.30811843f},
    {200.000f, 1.00306512f, 309.243487f, 9.79084298f, 4.57146778f, 2.34788446f},
    {250.000f, 0.99911397f, 308.765325f, 9.79068873f, 4.65390955f, 2.38765049f},
    {300.000f, 0.995162824f, 308.287162f, 9.79053448f, 4.73635132f, 2.42741652f},
    {350.000f, 0.991211678f, 307.809f, 9.79038024f, 4.81879309f, 2.46718255f},
    {400.000f, 0.987260531f, 307.330838f, 9.790226f, 4.90123486f, 2.50694858f},
    {450.000f, 0.983309385f, 306.852676f, 9.79007177f, 4.98367663f, 2.54671461f},
    {500.000f, 0.979358239f, 306.374513f, 9.78991754f, 5.06611841f, 2.58648064f},
    {550.000f, 0.975407093f, 305.896351f, 9.78976331f, 5.14856018f, 2.62624667f},
    {600.000f, 0.971455946f, 305.418189f, 9.78960909f, 5.23100195f, 2.66601271f},
    {650.000f, 0.96761437f, 304.929153f, 9.78945487f, 5.28523518f, 2.68721005f},
    {700.000f, 0.963832245f, 304.434217f, 9.78930065f, 5.32416273f, 2.69833221f},
    {750.000f, 0.960050121f, 303.939281f, 9.78914644f, 5.36309028f, 2.70945437f},
    {800.000f, 0.956267996f, 303.444345f, 9.78899223f, 5.40201784f, 2.72057652f},
    {850.000f, 0.952485872f, 302.949409f, 9.78883802f, 5.44094539f, 2.73169868f},
    {900.000f, 0.948703747f, 302.454473f, 9.78868382f, 5.47987294f, 2.74282084f},
    {950.000f, 0.944921622f, 301.959537f, 9.78852962f, 5.51880049f, 2.753943f},
    {1000.000f, 0.941139498f, 301.464601f, 9.78837542f, 5.55772804f, 2.76506515f},
    {1050.000f, 0.937357373f, 300.969665f, 9.78822123f, 5.59665559f, 2.77618731f},
    {1100.000f, 0.933575249f, 300.474729f, 9.78806704f, 5.63558314f, 2.78730947f},
    {1150.000f, 0.929793124f, 299.979793f, 9.78791286f, 5.67451069f, 2.79843163f},
    {1200.000f, 0.92609684f, 299.500611f, 9.78775868f, 5.67331306f, 2.80557245f},
    {1250.000f, 0.922414649f, 299.024015f, 9.7876045f, 5.66552838f, 2.81205968f},
    {1300.000f, 0.918732457f, 298.54742f, 9.78745032f, 5.6577437f, 2.81854692f},
    {1350.000f, 0.915050265f, 298.070824f, 9.78729615f, 5.64995902f, 2.82503415f},
    {1400.000f, 0.911368073f, 297.594229f, 9.78714198f, 5.64217434f, 2.83152138f},
    {1450.000f, 0.907685882f, 297.117633f, 9.78698782f, 5.63438966f, 2.83800862f},
    {1500.000f, 0.90400369f, 296.641038f, 9.78683366f, 5.62660498f, 2.84449585f},
    {1550.000f, 0.900321498f, 296.164442f, 9.7866795f, 5.61882029f, 2.85098309f},
    {1600.000f, 0.896639306f, 295.687847f, 9.78652535f, 5.61103561f, 2.85747032f},
    {1650.000f, 0.892957114f, 295.211252f, 9.7863712f, 5.60325093f, 2.86395756f},
    {1700.000f, 0.889274923f, 294.734656f, 9.78621706f, 5.59546625f, 2.87044479f},
    {1750.000f, 0.885592731f, 294.258061f, 9.78606291f, 5.58768157f, 2.87693203f},
    {1800.000f, 0.881910539f, 293.781465f, 9.78590878f, 5.57989689f, 2.88341926f},
    {1850.000f, 0.878228347f, 293.30487f, 9.78575464f, 5.57211221f, 2.88990649f},
    {1900.000f, 0.874546156f, 292.828274f, 9.78560051f, 5.56432753f, 2.89639373f},
    {1950.000f, 0.870863964f, 292.351679f, 9.78544638f, 5.55654284f, 2.90288096f},
    {2000.000f, 0.867181772f, 291.875083f, 9.78529226f, 5.54875816f, 2.9093682f},
    {2050.000f, 0.86349958f, 291.398488f, 9.78513814f, 5.54097348f, 2.91585543f},
    {2100.000f, 0.859817389f, 290.921892f, 9.78498402f, 5.5331888f, 2.92234267f},
    {2150.000f, 0.856135197f, 290.445297f, 9.78482991f, 5.52540412f, 2.9288299f},
    {2200.000f, 0.852453005f, 289.968701f, 9.7846758f, 5.51761944f, 2.93531714f},
    {2250.000f, 0.848770813f, 289.492106f, 9.78452169f, 5.50983476f, 2.94180437f},
    {2300.000f, 0.845088622f, 289.01551f, 9.78436759f, 5.50205008f, 2.9482916f},
    {2350.000f, 0.841541099f, 288.551177f, 9.78421349f, 5.4819103f, 2.9003251f},
    {2400.000f, 0.838041719f, 288.091227f, 9.7840594f, 5.45735367f, 2.83289182f},
    {2450.000f, 0.834542339f, 287.631278f, 9.78390531f, 5.43279704f, 2.76545855f},
    {2500.000f, 0.831042959f, 287.171328f, 9.78375122f, 5.40824042f, 2.69802527f},
    {2550.000f, 0.82754358f, 286.711379f, 9.78359714f, 5.38368379f, 2.63059199f},
    {2600.000f, 0.8240442f, 286.251429f, 9.78344306f, 5.35912716f, 2.56315872f},
    {2650.000f, 0.82054482f, 285.79148f, 9.78328898f, 5.33457054f, 2.49572544f},
    {2700.000f, 0.81704544f, 285.33153f, 9.78313491f, 5.31001391f, 2.42829217f},
    {2750.000f, 0.813546061f, 284.871581f, 9.78298084f, 5.28545728f, 2.36085889f},
    {2800.000f, 0.810046681f, 284.411631f, 9.78282677f, 5.26090066f, 2.29342561f},
    {2850.000f, 0.806547301f, 283.951682f, 9.78267271f, 5.23634403f, 2.22599234f},
    {2900.000f, 0.803047921f, 283.491732f, 9.78251865f, 5.2117874f, 2.15855906f},
    {2950.000f, 0.799548542f, 283.031783f, 9.78236459f, 5.18723078f, 2.09112579f},
    {3000.000f, 0.796049162f, 282.571833f, 9.78221054f, 5.16267415f, 2.02369251f},
    {3050.000f, 0.792549782f, 282.111884f, 9.78205649f, 5.13811752f, 1.95625923f},
    {3100.000f, 0.789050402f, 281.651934f, 9.78190245f, 5.1135609f, 1.88882596f},
    {3150.000f, 0.785551022f, 281.191985f, 9.78174841f, 5.08900427f, 1.82139268f},
    {3200.000f, 0.782051643f, 280.732035f, 9.78159437f, 5.06444765f, 1.75395941f},
    {3250.000f, 0.778552263f, 280.272086f, 9.78144034f, 5.03989102f, 1.68652613f},
    {3300.000f, 0.775052883f, 279.812136f, 9.78128631f, 5.01533439f, 1.61909285f},
    {3350.000f, 0.771553503f, 279.352187f, 9.78113228f, 4.99077777f, 1.55165958f},
    {3400.000f, 0.768054124f, 278.892237f, 9.78097826f, 4.96622114f, 1.4842263f},
    {3450.000f, 0.764554744f, 278.432288f, 9.78082424f, 4.94166451f, 1.41679303f},
    {3500.000f, 0.761055364f, 277.972338f, 9.78067023f, 4.91710789f, 1.34935975f},
    {3550.000f, 0.757555984f, 277.512389f, 9.78051621f, 4.89255126f, 1.28192647f},
    {3600.000f, 0.754060432f, 277.059904f, 9.78036221f, 4.86778208f, 1.21727674f},
    {3650.000f, 0.75060792f, 276.691361f, 9.7802082f, 4.8406227f, 1.18392914f},
    {3700.000f, 0.747155408f, 276.322819f, 9.7800542f, 4.81346331f, 1.15058153f},
    {3750.000f, 0.743702896f, 275.954276f, 9.7799002f, 4.78630393f, 1.11723393f},
    {3800.000f, 0.740250384f, 275.585733f, 9.77974621f, 4.75914454f, 1.08388633f},
    {3850.000f, 0.736797872f, 275.217191f, 9.77959222f, 4.73198515f, 1.05053873f},
    {3900.000f, 0.73334536f, 274.848648f, 9.77943823f, 4.70482577f, 1.01719113f},
    {3950.000f, 0.729892848f, 274.480106f, 9.77928425f, 4.67766638f, 0.98384353f},
    {4000.000f, 0.726440336f, 274.111563f, 9.77913027f, 4.65050699f, 0.950495929f},
};

/// Copies @p n floats from @p src to @p dst.
static void copyTable(float* dst, const float* src, int n) {
    for (int i = 0; i < n; ++i) dst[i] = src[i];
}

} // namespace

void loadControlFreakRocketConfig(RocketConfig& cfg) {
    // Values copied from FV-Controls/rockets/Control Freak/Rocketpy/rocketpy.ipynb.
    cfg.m_0    = 22.2940008f;
    cfg.m_f    = 18.6420000f;
    cfg.m_prop = 3.65200077f;

    cfg.I_0   = 13.1572813f;
    cfg.I_f   = 10.3627008f;
    cfg.I_3_0 = 0.0648822543f;
    cfg.I_3_f = 0.0613600000f;

    cfg.x_CG_0 = 1.84913484f;
    cfg.x_CG_f = 1.70762222f;

    cfg.d    = 0.13081f;
    cfg.L_ne = 2.870f;

    cfg.t_rail    = 0.309f;
    cfg.t_burnout = 3.28f;
    cfg.t_apogee  = 25.678f;

    cfg.base_cn   = 10.581f;
    cfg.base_cp   = 2.17842618f;
    cfg.canard_cn = 0.857f;
    cfg.canard_cp = 1.049f;
    cfg.canard_plane_angle_deg = 0.0f;

    cfg.N_fins      = 4;
    cfg.Cr          = 0.305f;
    cfg.Ct          = 0.152f;
    cfg.s           = 0.133f;
    cfg.delta_deg   = 0.0f;
    cfg.Cnalpha_fin = 2.34275f;

    cfg.canard_moment_coeff_per_deg = -4.23e-7f;
    cfg.roll_damping_lambda = 30.0f;
    cfg.max_deflection_rad  = 9.0f * (M_PI / 180.0f);
    cfg.max_deflection_rate = 428.571428571f * (M_PI / 180.0f);
    cfg.min_control_speed   = 30.0f;
    cfg.irec_compliant      = true;

    cfg.Q_angular[0] = 2.0e-4f;
    cfg.Q_angular[1] = 2.0e-4f;
    cfg.Q_angular[2] = 3.0e-4f;
    cfg.Q_velocity[0] = 2.5e-3f;
    cfg.Q_velocity[1] = 2.5e-3f;
    cfg.Q_velocity[2] = 2.5e-3f;
    for (int i = 0; i < 4; ++i) cfg.Q_quat[i] = 1.0e-5f;

    cfg.accel_noise_std_g = 0.5f;
    cfg.gyro_noise_std    = 0.01f;
    cfg.accel_model_std_g = 0.1f;
    cfg.accel_burn_std_g  = 2.0f;
    cfg.P0_scale          = 0.1f;

    cfg.rho_0    = 1.213f;
    cfg.g_0      = 9.8014f;
    cfg.v_wind_x = 0.0f;
    cfg.v_wind_y = 0.0f;

    for (int i = 0; i < 10; ++i) cfg.x0[i] = 0.0f;
    // Matches FV-Controls setSimParamsFromRailAngle(45 deg, nose_to_tail):
    // launch roll angle = 360 - 45 = 315 deg.
    cfg.x0[6] = -0.923879533f;
    cfg.x0[9] =  0.382683432f;

    cfg.rem_enabled             = true;
    cfg.rem_post_burn_delay_s   = 0.25f;
    cfg.rem_min_cmd_rad         = 2.0f * (M_PI / 180.0f);
    cfg.rem_min_expected_accel  = 0.3f;
    cfg.rem_min_measured_accel  = 0.3f;
    cfg.rem_required_mismatches = 5;
    cfg.rem_allow_flip_back     = true;

    cfg.thrust_n = static_cast<int>(sizeof(kThrustT) / sizeof(kThrustT[0]));
    copyTable(cfg.thrust_t, kThrustT, cfg.thrust_n);
    copyTable(cfg.thrust_N, kThrustN, cfg.thrust_n);

    cfg.drag_on_n = static_cast<int>(sizeof(kDragMach) / sizeof(kDragMach[0]));
    copyTable(cfg.drag_on_mach, kDragMach, cfg.drag_on_n);
    copyTable(cfg.drag_on_cd, kDragCd, cfg.drag_on_n);

    cfg.drag_off_n = static_cast<int>(sizeof(kDragMach) / sizeof(kDragMach[0]));
    copyTable(cfg.drag_off_mach, kDragMach, cfg.drag_off_n);
    copyTable(cfg.drag_off_cd, kDragCd, cfg.drag_off_n);
}

// ─── inv6: LU decomposition with partial pivoting ────────────────────────────
Mat<6,6> inv6(const Mat<6,6>& A) {
    Mat<6,6> LU = A;
    int piv[6] = {0,1,2,3,4,5};

    for (int k = 0; k < 6; k++) {
        // partial pivot
        int   mx  = k;
        float mxv = fabsf(LU(k,k));
        for (int i = k+1; i < 6; i++) {
            if (fabsf(LU(i,k)) > mxv) { mxv = fabsf(LU(i,k)); mx = i; }
        }
        if (mx != k) {
            for (int j = 0; j < 6; j++) { float t = LU(k,j); LU(k,j)=LU(mx,j); LU(mx,j)=t; }
            int  tp = piv[k]; piv[k]=piv[mx]; piv[mx]=tp;
        }
        if (fabsf(LU(k,k)) < 1e-12f) return eye<6>(); // singular fallback
        for (int i = k+1; i < 6; i++) {
            LU(i,k) /= LU(k,k);
            for (int j = k+1; j < 6; j++) LU(i,j) -= LU(i,k)*LU(k,j);
        }
    }

    Mat<6,6> inv;
    for (int col = 0; col < 6; col++) {
        float b[6] = {};
        b[col] = 1.0f;
        // apply row permutation
        for (int k = 0; k < 6; k++) {
            if (piv[k] != k) { float t = b[piv[k]]; b[piv[k]]=b[k]; b[k]=t; }
        }
        // forward substitution (unit lower triangular L)
        for (int i = 1; i < 6; i++)
            for (int j = 0; j < i; j++) b[i] -= LU(i,j)*b[j];
        // back substitution (upper triangular U)
        for (int i = 5; i >= 0; i--) {
            for (int j = i+1; j < 6; j++) b[i] -= LU(i,j)*b[j];
            b[i] /= LU(i,i);
        }
        for (int i = 0; i < 6; i++) inv(i,col) = b[i];
    }
    return inv;
}

// ─── lerp ─────────────────────────────────────────────────────────────────────
float lerp(const float* xs, const float* ys, int n, float xi) {
    if (n <= 0) return 0.0f;
    if (xi <= xs[0]) return ys[0];
    if (xi >= xs[n-1]) return ys[n-1];
    // binary search for bracket
    int lo = 0, hi = n-1;
    while (hi - lo > 1) {
        int mid = (lo+hi) >> 1;
        if (xs[mid] <= xi) lo = mid; else hi = mid;
    }
    float t = (xi - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + t * (ys[hi] - ys[lo]);
}

// ─── Physics constructor ──────────────────────────────────────────────────────
Physics::Physics(const RocketConfig& c) : cfg(c) {
    // Match FV-Controls/src/dynamics/momentsforces.py roll EOM.
    const float gamma = (cfg.Cr > 0.0f) ? (cfg.Ct / cfg.Cr) : 0.0f;
    r_t = cfg.d * 0.5f;
    tau = (r_t > 0.0f) ? ((cfg.s + r_t) / r_t) : 1.0f;

    y_ma = (cfg.s / 3.0f) * (1.0f + 2.0f*gamma) / (1.0f + gamma);

    const float tau2 = tau * tau;
    const float tau_minus_1 = tau - 1.0f;
    const float asin_arg = (tau2 - 1.0f) / (tau2 + 1.0f);
    const float asin_term = asinf(fmaxf(-1.0f, fminf(1.0f, asin_arg)));
    const float shape_f =
        (1.0f / (M_PI * M_PI)) *
        ((M_PI*M_PI / 4.0f) * ((tau + 1.0f)*(tau + 1.0f) / tau2)
       + (M_PI * (tau2 + 1.0f)*(tau2 + 1.0f) / (tau2 * tau_minus_1*tau_minus_1)) * asin_term
       - (2.0f * M_PI * (tau + 1.0f)) / (tau * tau_minus_1)
       + ((tau2 + 1.0f)*(tau2 + 1.0f) / (tau2 * tau_minus_1*tau_minus_1)) * asin_term*asin_term
       - (4.0f * (tau + 1.0f) / (tau * tau_minus_1)) * asin_term
       + (8.0f / (tau_minus_1*tau_minus_1)) * logf((tau2 + 1.0f) / (2.0f * tau)));

    // Runtime uses M_f_roll = K_f * q_dyn.
    K_f = shape_f * (float)cfg.N_fins * (y_ma + r_t) * cfg.Cnalpha_fin
          * cfg.delta_rad * cfg.A_ref;

    const float trap_integral = cfg.s / 12.0f *
        ((cfg.Cr + 3.0f*cfg.Ct) * cfg.s*cfg.s
       + 4.0f * (cfg.Cr + 2.0f*cfg.Ct) * cfg.s * r_t
       + 6.0f * (cfg.Cr + cfg.Ct) * r_t*r_t);
    C_ldw = 2.0f * (float)cfg.N_fins * cfg.Cnalpha_fin
            / (cfg.A_ref * cfg.d*cfg.d) * cosf(cfg.delta_rad) * trap_integral;
    const float shape_d = 1.0f +
        (((tau - gamma) / tau) - ((1.0f - gamma) / tau_minus_1) * logf(tau)) /
        (((tau + 1.0f) * (tau - gamma) / 2.0f)
       - ((1.0f - gamma) * (tau*tau*tau - 1.0f) / (3.0f * tau_minus_1)));

    // Runtime uses M_d_roll = 0.5*rho*v_mag*K_d*w3.
    K_d = shape_d * cfg.A_ref * cfg.d * C_ldw * (cfg.d * 0.5f);
}

// ─── Time-varying parameters ──────────────────────────────────────────────────
float Physics::getMass(float t) const {
    if (t >= cfg.t_burnout) return cfg.m_f;
    return cfg.m_0 - cfg.m_prop * (t / cfg.t_burnout);
}

float Physics::getInertiaPitch(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_f;
    return cfg.I_0 + (cfg.I_f - cfg.I_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaRoll(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_3_f;
    return cfg.I_3_0 + (cfg.I_3_f - cfg.I_3_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaDotPitch(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_f - cfg.I_0) / cfg.t_burnout;
}

float Physics::getInertiaDotRoll(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_3_f - cfg.I_3_0) / cfg.t_burnout;
}

float Physics::getCG(float t) const {
    if (t >= cfg.t_burnout) return cfg.x_CG_f;
    return cfg.x_CG_0 + (cfg.x_CG_f - cfg.x_CG_0) * (t / cfg.t_burnout);
}

float Physics::getThrust(float t) const {
    if (t < 0.0f || t > cfg.t_burnout || cfg.thrust_n == 0) return 0.0f;
    if (t < cfg.thrust_t[0] || t > cfg.thrust_t[cfg.thrust_n - 1]) return 0.0f;
    return lerp(cfg.thrust_t, cfg.thrust_N, cfg.thrust_n, t);
}

float Physics::getDragCoeff(float mach, float t) const {
    if (t < cfg.t_burnout && cfg.drag_on_n > 0)
        return lerp(cfg.drag_on_mach, cfg.drag_on_cd, cfg.drag_on_n, mach);
    if (cfg.drag_off_n > 0)
        return lerp(cfg.drag_off_mach, cfg.drag_off_cd, cfg.drag_off_n, mach);
    return 0.45f; // fallback
}

float Physics::speedOfSound(float T_kelvin) {
    return sqrtf(1.4f * 287.05f * T_kelvin);
}

WeatherSample Physics::weatherAtAltitude(float alt_m) const {
    static constexpr int n = static_cast<int>(sizeof(kWeatherProfile) / sizeof(kWeatherProfile[0]));
    if (n <= 0) {
        WeatherSample fallback;
        fallback.rho = cfg.rho_0;
        fallback.temperature_K = 288.15f;
        fallback.g = cfg.g_0;
        fallback.wind_x = cfg.v_wind_x;
        fallback.wind_y = cfg.v_wind_y;
        return fallback;
    }

    auto sampleFrom = [](const WeatherPoint& p) {
        WeatherSample s;
        s.rho = p.rho;
        s.temperature_K = p.temperature_K;
        s.g = p.g;
        s.wind_x = p.wind_x;
        s.wind_y = p.wind_y;
        return s;
    };

    if (alt_m <= kWeatherProfile[0].altitude_agl_m) return sampleFrom(kWeatherProfile[0]);
    if (alt_m >= kWeatherProfile[n - 1].altitude_agl_m) return sampleFrom(kWeatherProfile[n - 1]);

    int lo = 0;
    int hi = n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) >> 1;
        if (kWeatherProfile[mid].altitude_agl_m <= alt_m) lo = mid;
        else hi = mid;
    }

    const WeatherPoint& a = kWeatherProfile[lo];
    const WeatherPoint& b = kWeatherProfile[hi];
    const float u = (alt_m - a.altitude_agl_m) / (b.altitude_agl_m - a.altitude_agl_m);

    WeatherSample s;
    s.rho           = a.rho           + u * (b.rho           - a.rho);
    s.temperature_K = a.temperature_K + u * (b.temperature_K - a.temperature_K);
    s.g             = a.g             + u * (b.g             - a.g);
    s.wind_x        = a.wind_x        + u * (b.wind_x        - a.wind_x);
    s.wind_y        = a.wind_y        + u * (b.wind_y        - a.wind_y);
    return s;
}

// ─── R_BW: world-to-body DCM from unit quaternion ────────────────────────────
void Physics::R_BW(float qw, float qx, float qy, float qz, float R[3][3]) {
    const float qx2=qx*qx, qy2=qy*qy, qz2=qz*qz;
    R[0][0] = 1.0f - 2.0f*(qy2+qz2);   R[0][1] = 2.0f*(qx*qy+qw*qz);    R[0][2] = 2.0f*(qx*qz-qw*qy);
    R[1][0] = 2.0f*(qx*qy-qw*qz);      R[1][1] = 1.0f - 2.0f*(qx2+qz2); R[1][2] = 2.0f*(qy*qz+qw*qx);
    R[2][0] = 2.0f*(qx*qz+qw*qy);      R[2][1] = 2.0f*(qy*qz-qw*qx);    R[2][2] = 1.0f - 2.0f*(qx2+qy2);
}

// ─── Air-relative velocity in body frame ─────────────────────────────────────
void Physics::airRelativeVelocity(const StateVec& x, float v_air[3]) const {
    airRelativeVelocity(x, cfg.v_wind_x, cfg.v_wind_y, v_air);
}

void Physics::airRelativeVelocity(const StateVec& x, float wind_x, float wind_y,
                                  float v_air[3]) const {
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);
    // rotate to body frame
    float w_body[3] = {
        R[0][0]*wind_x + R[0][1]*wind_y,
        R[1][0]*wind_x + R[1][1]*wind_y,
        R[2][0]*wind_x + R[2][1]*wind_y
    };
    v_air[0] = x(3,0) - w_body[0];
    v_air[1] = x(4,0) - w_body[1];
    v_air[2] = x(5,0) - w_body[2];
}

// ─── Canard moment (CFD numeric) ──────────────────────────────────────────────
float Physics::canardMoment(float v_mag, float zeta_rad) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag
           * (zeta_rad * (180.0f / M_PI));
}

float Physics::canardMomentJacobian(float v_mag) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag * (180.0f / M_PI);
}

// ─── Full nonlinear EOM ───────────────────────────────────────────────────────
StateVec Physics::eom(float t, const StateVec& x, float u_zeta, float alt) const {
    // Extract state
    float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    // Normalize quaternion
    float qn = sqrtf(qw*qw+qx*qx+qy*qy+qz*qz);
    if (qn > 1e-8f) { qw/=qn; qx/=qn; qy/=qn; qz/=qn; }

    // Time-varying params
    const float m    = getMass(t);
    const float I1   = getInertiaPitch(t);
    const float I3   = getInertiaRoll(t);
    const float I1d  = getInertiaDotPitch(t);
    const float I3d  = getInertiaDotRoll(t);
    const float xCG  = getCG(t);
    const float Thr  = getThrust(t);
    const WeatherSample wx = weatherAtAltitude(alt);

    // Rotation matrix
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);

    // Air-relative velocity
    float va[3];
    airRelativeVelocity(x, wx.wind_x, wx.wind_y, va);

    // Airspeed
    const float vmag2 = va[0]*va[0]+va[1]*va[1]+va[2]*va[2];
    const float vmag  = sqrtf(vmag2);
    static constexpr float EPS_AOA = 1.0e-9f;
    const float vmag_reg = sqrtf(vmag2 + EPS_AOA*EPS_AOA);
    const float v_xy_mag = sqrtf(va[0]*va[0] + va[1]*va[1] + EPS_AOA*EPS_AOA);

    // Angle of attack (from axial z-body), clamped to ±15 deg
    static constexpr float AoA_MAX = 15.0f * (M_PI / 180.0f);
    float alpha = 0.0f;
    if (vmag > 0.1f)
        alpha = atan2f(v_xy_mag, va[2] + EPS_AOA);
    if (alpha >  AoA_MAX) alpha =  AoA_MAX;
    if (alpha < -AoA_MAX) alpha = -AoA_MAX;

    // Directional canard-plane AoA, matching FV-Controls momentsforces.py.
    const float canard_lat = va[0]*cfg.canard_dir_1 + va[1]*cfg.canard_dir_2;
    const float canard_lat_mag = sqrtf(canard_lat*canard_lat + EPS_AOA*EPS_AOA);
    const float vcan2 = canard_lat*canard_lat + va[2]*va[2];
    const float vcan_mag = sqrtf(vcan2 + EPS_AOA*EPS_AOA);
    float alpha_can = 0.0f;
    if (vcan_mag > 0.1f)
        alpha_can = atan2f(canard_lat_mag, va[2] + EPS_AOA);
    if (alpha_can >  AoA_MAX) alpha_can =  AoA_MAX;
    if (alpha_can < -AoA_MAX) alpha_can = -AoA_MAX;

    // Mach, Cd, dynamic pressure
    const float c_snd = speedOfSound(wx.temperature_K);
    const float mach  = (c_snd > 0.0f) ? vmag / c_snd : 0.0f;
    const float Cd    = getDragCoeff(mach, t);
    const float rho   = wx.rho;
    const float q_dyn = 0.5f * rho * vmag2;

    // Rail: no lift/moments before release. Thrust, gravity, and drag still act,
    // matching FV-Controls' force model.
    const bool on_rail = (t < cfg.t_rail);

    // ── Forces (body frame) ──────────────────────────────────────────────────
    float F[3] = {0.0f, 0.0f, 0.0f};

    // Gravity: g_world = [0,0,-g]; g_body = R_BW * g_world = -g * R[:,2]
    F[0] += m * (-wx.g * R[0][2]);
    F[1] += m * (-wx.g * R[1][2]);
    F[2] += m * (-wx.g * R[2][2]);

    // Thrust along +z body axis
    F[2] += Thr;

    // Aerodynamic drag (along -v_air)
    if (vmag > 0.1f) {
        const float D = Cd * cfg.A_ref * q_dyn / vmag_reg;
        F[0] -= D * va[0];
        F[1] -= D * va[1];
        F[2] -= D * va[2];
    }

    if (!on_rail && vmag > 0.1f) {
        // Symmetric base-rocket normal force.
        const float L_base = q_dyn * cfg.base_cn * alpha * cfg.A_ref;
        const float cos_a = cosf(alpha);
        F[0] += L_base * (-cos_a * va[0] / v_xy_mag);
        F[1] += L_base * (-cos_a * va[1] / v_xy_mag);
        F[2] += L_base * sinf(alpha);

        // Two-canard normal force acts only in the configured canard plane.
        const float q_can = 0.5f * rho * vcan2;
        const float L_can = q_can * cfg.canard_cn * alpha_can * cfg.A_ref;
        const float cos_a_can = cosf(alpha_can);
        const float canard_lat_sign = canard_lat / canard_lat_mag;
        F[0] += L_can * (-cos_a_can * canard_lat_sign * cfg.canard_dir_1);
        F[1] += L_can * (-cos_a_can * canard_lat_sign * cfg.canard_dir_2);
        F[2] += L_can * sinf(alpha_can);
    }

    // ── Moments (body frame) ─────────────────────────────────────────────────
    float M[3] = {0.0f, 0.0f, 0.0f};

    if (!on_rail) {
        // Corrective moments: symmetric base plus directional canard plane.
        if (vmag > 0.1f) {
            const float cos_a = va[2] / vmag_reg;
            const float C_raw_base = vmag2 * cfg.A_ref * cfg.base_cn * alpha
                                   * (cfg.base_cp - xCG) * rho * 0.5f;
            M[0] += -C_raw_base * cos_a * va[1] / v_xy_mag;
            M[1] +=  C_raw_base * cos_a * va[0] / v_xy_mag;

            const float cos_a_can = va[2] / vcan_mag;
            const float C_raw_can = vcan2 * cfg.A_ref * cfg.canard_cn * alpha_can
                                  * (cfg.canard_cp - xCG) * rho * 0.5f;
            const float canard_lat_sign = canard_lat / canard_lat_mag;
            M[0] += -C_raw_can * cos_a_can * canard_lat_sign * cfg.canard_dir_2;
            M[1] +=  C_raw_can * cos_a_can * canard_lat_sign * cfg.canard_dir_1;
        }

        // Propulsive and aerodynamic pitch/yaw damping from FV-Controls.
        const float mdot = (cfg.t_burnout > 0.0f) ? (cfg.m_prop / cfg.t_burnout) : 0.0f;
        const float Cdp = (t < cfg.t_burnout)
                        ? mdot * (cfg.L_ne - xCG) * (cfg.L_ne - xCG)
                        : 0.0f;
        const float Cda_base = 0.5f * rho * vmag_reg * cfg.A_ref
                             * cfg.base_cn * (cfg.base_cp - xCG) * (cfg.base_cp - xCG);
        const float Cda_can = 0.5f * rho * vcan_mag * cfg.A_ref
                            * cfg.canard_cn * (cfg.canard_cp - xCG) * (cfg.canard_cp - xCG);
        M[0] -= (Cdp + Cda_base + Cda_can * cfg.canard_dir_2 * cfg.canard_dir_2) * w1;
        M[1] -= (Cdp + Cda_base + Cda_can * cfg.canard_dir_1 * cfg.canard_dir_1) * w2;

        // Fin cant roll forcing (proportional to dynamic pressure)
        M[2] += K_f * q_dyn;

        // Roll aerodynamic damping: M = -0.5 * rho * v * K_d * w3
        M[2] -= 0.5f * rho * vmag * K_d * w3;

        // Canard roll moment (CFD numeric)
        M[2] += canardMoment(vmag, u_zeta);
    }

    // ── Euler rigid-body (I1=I2 for symmetric rocket) ────────────────────────
    const float I2 = I1;
    StateVec xdot;
    xdot(0,0) = ((I2-I3)*w2*w3 + M[0] - I1d*w1) / I1;
    xdot(1,0) = ((I3-I1)*w3*w1 + M[1] - I1d*w2) / I1;
    xdot(2,0) = ((I1-I2)*w1*w2 + M[2] - I3d*w3) / I3;

    // ── Velocity: vdot = F/m - omega×v ───────────────────────────────────────
    xdot(3,0) = F[0]/m - (w2*v3 - w3*v2);
    xdot(4,0) = F[1]/m - (w3*v1 - w1*v3);
    xdot(5,0) = F[2]/m - (w1*v2 - w2*v1);

    // ── Quaternion kinematics: qdot = 0.5 * Omega(w) * q ─────────────────────
    xdot(6,0) = 0.5f*(-w1*qx - w2*qy - w3*qz);
    xdot(7,0) = 0.5f*( w1*qw + w3*qy - w2*qz);
    xdot(8,0) = 0.5f*( w2*qw - w3*qx + w1*qz);
    xdot(9,0) = 0.5f*( w3*qw + w2*qx - w1*qy);

    return xdot;
}

// ─── Numerical Jacobian A = df/dx (central finite differences) ───────────────
StateMat Physics::jacobianA(float t, const StateVec& x, float u, float alt) const {
    static constexpr float EPS = 1e-5f;
    StateMat A;
    for (int j = 0; j < 10; j++) {
        StateVec xp = x, xm = x;
        xp(j,0) += EPS;
        xm(j,0) -= EPS;
        StateVec fp = eom(t, xp, u, alt);
        StateVec fm = eom(t, xm, u, alt);
        for (int i = 0; i < 10; i++)
            A(i,j) = (fp(i,0) - fm(i,0)) / (2.0f * EPS);
    }
    return A;
}

// ─── Analytic B = df/du ───────────────────────────────────────────────────────
// Only roll row (index 2) is nonzero: dw3dot/du = dM_canard/du / I3
InputMat Physics::jacobianB(float t, const StateVec& x) const {
    float va[3];
    airRelativeVelocity(x, va);
    const float vmag = sqrtf(va[0]*va[0]+va[1]*va[1]+va[2]*va[2]);
    const float I3   = getInertiaRoll(t);
    InputMat B;
    B(2,0) = (I3 > 1e-10f) ? (canardMomentJacobian(vmag) / I3) : 0.0f;
    return B;
}

// ─── Measurement Jacobian C = dy/dx (6×10) ───────────────────────────────────
MeasMat Physics::jacobianC(float t, const StateVec& x, float u,
                            const StateMat& A, float alt) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    const WeatherSample wx = weatherAtAltitude(alt);
    const float g_body_mag = wx.g;
    const float accel_scale = cfg.g_0;

    // J_cross = ∂(ω×v)/∂x  [3×10]
    // ω×v = [w2*v3-w3*v2, w3*v1-w1*v3, w1*v2-w2*v1]
    Mat<3,10> Jcross;
    // ∂/∂w1: [0, -v3, v2]
    Jcross(0,0)=0.0f;  Jcross(1,0)=-v3;   Jcross(2,0)= v2;
    // ∂/∂w2: [v3, 0, -v1]
    Jcross(0,1)= v3;   Jcross(1,1)=0.0f;  Jcross(2,1)=-v1;
    // ∂/∂w3: [-v2, v1, 0]
    Jcross(0,2)=-v2;   Jcross(1,2)= v1;   Jcross(2,2)=0.0f;
    // ∂/∂v1: [0, w3, -w2]
    Jcross(0,3)=0.0f;  Jcross(1,3)= w3;   Jcross(2,3)=-w2;
    // ∂/∂v2: [-w3, 0, w1]
    Jcross(0,4)=-w3;   Jcross(1,4)=0.0f;  Jcross(2,4)= w1;
    // ∂/∂v3: [w2, -w1, 0]
    Jcross(0,5)= w2;   Jcross(1,5)=-w1;   Jcross(2,5)=0.0f;
    // ∂/∂q = 0 (columns 6-9 already zero)

    // J_g = ∂(g_body)/∂x  [3×10]
    // g_body = R_BW * [0,0,-g]; only q-columns nonzero
    // R_BW col-2: [2(qx*qz-qw*qy), 2(qy*qz+qw*qx), 1-2(qx²+qy²)]
    Mat<3,10> Jg;
    // ∂g_body/∂qw
    // g_body[0] = -g * 2*(qx*qz - qw*qy)
    // ∂g_body[0]/∂qw = -g * 2*(-qy) = 2*g*qy
    Jg(0,6) = 2.0f*g_body_mag*qy;
    Jg(0,7) = -2.0f*g_body_mag*qz;         // ∂/∂qx = -g*2*qz
    Jg(0,8) = 2.0f*g_body_mag*qw;          // ∂/∂qy = -g*2*(-qw) = 2*g*qw
    Jg(0,9) = -2.0f*g_body_mag*qx;         // ∂/∂qz = -g*2*qx

    // g_body[1] = -g * 2*(qy*qz + qw*qx)
    Jg(1,6) = -2.0f*g_body_mag*qx;         // ∂/∂qw = -g*2*qx
    Jg(1,7) = -2.0f*g_body_mag*qw;         // ∂/∂qx = -g*2*qw
    Jg(1,8) = -2.0f*g_body_mag*qz;         // ∂/∂qy = -g*2*qz
    Jg(1,9) = -2.0f*g_body_mag*qy;         // ∂/∂qz = -g*2*qy

    // g_body[2] = -g * (1 - 2*(qx²+qy²))
    // ∂/∂qw = 0
    Jg(2,7) = 4.0f*g_body_mag*qx;          // ∂/∂qx = -g*(-4*qx) = 4*g*qx
    Jg(2,8) = 4.0f*g_body_mag*qy;          // ∂/∂qy = 4*g*qy

    // Build C matrix
    MeasMat C;

    // Accel rows 0-2: C[0:3,:] = (A[3:6,:] + J_cross - J_g) / accel_scale
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 10; j++) {
            C(i, j) = (A(3+i, j) + Jcross(i, j) - Jg(i, j)) / accel_scale;
        }
    }
    // Zero angular-rate columns on accel rows (accel doesn't directly observe w)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C(i, j) = 0.0f;

    // Gyro rows 3-5: identity on angular-rate columns
    C(3,0) = 1.0f;
    C(4,1) = 1.0f;
    C(5,2) = 1.0f;

    return C;
}

// ─── Measurement prediction ───────────────────────────────────────────────────
// Returns [accel_g (3), gyro_rad_s (3)]
MeasVec Physics::predictMeasurement(float t, const StateVec& x, float u,
                                    float alt) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    const WeatherSample wx = weatherAtAltitude(alt);
    StateVec xdot = eom(t, x, u, alt);

    // vdot (body) = xdot[3:6]
    // Specific force = vdot + omega×v - g_body, divided by g to get g units
    // omega × v
    float oxv[3] = {
        w2*v3 - w3*v2,
        w3*v1 - w1*v3,
        w1*v2 - w2*v1
    };

    // g_body = R_BW * [0,0,-g]
    float R[3][3];
    Physics::R_BW(qw, qx, qy, qz, R);
    float g_body[3] = {
        R[0][2] * (-wx.g),
        R[1][2] * (-wx.g),
        R[2][2] * (-wx.g)
    };

    MeasVec y;
    for (int i = 0; i < 3; i++)
        y(i, 0) = (xdot(3+i, 0) + oxv[i] - g_body[i]) / cfg.g_0;

    // Gyro: direct angular rate observation
    y(3,0) = w1;
    y(4,0) = w2;
    y(5,0) = w3;

    return y;
}
