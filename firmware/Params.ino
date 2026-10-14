// Tunable parameter table and flash (NVS) storage.
// Adding a parameter: add the field to Params in Config.h and one line here; the serial,
// UDP and web interfaces pick it up automatically. Bump PARAMS_VERSION when the struct
// layout changes so old flash contents are ignored instead of misread.

static const uint32_t PARAMS_VERSION = 1;

#define PARAM_ROW(field, lo, hi, def, desc) {#field, &P.field, lo, hi, def, desc}

const ParamInfo PARAM_TABLE[] = {
  PARAM_ROW(kp,          0.0f,   50.0f,  6.0f,   "Açı P [%/°]: eğim başına motor gücü"),
  PARAM_ROW(ki,          0.0f,  500.0f, 100.0f,  "Açı I [%/(°·s)]: tekerlek hız komutu; motor hız kaynağı gibi davrandığı için şart"),
  PARAM_ROW(kd,          0.0f,    5.0f,  0.4f,   "Açı D [%/(°/s)]: devrilme hızına karşı sönüm (fazlası 20 Hz titretir)"),
  PARAM_ROW(kv,          0.0f,    1.0f,  0.1f,   "Hız geri beslemesi [°/%]: kaymayı durdurmak için ters eğilme"),
  PARAM_ROW(kx,          0.0f,    1.0f,  0.1f,   "Konum geri beslemesi [°/(%·s)]: başlangıç noktasına dönme"),
  PARAM_ROW(kxi,         0.0f,    0.2f,  0.005f, "Denge açısı öğrenme hızı (konum döngüsünün I terimi)"),
  PARAM_ROW(max_lean,    1.0f,   15.0f,  6.0f,   "Konum döngüsünün isteyebileceği en büyük eğim [°]"),
  PARAM_ROW(trim,      -15.0f,   15.0f,  3.0f,   "Denge açısı [°]: trimcal ile ölçülür, sonra kendisi öğrenir"),
  PARAM_ROW(vel_tau,     0.005f,  0.5f,  0.1f,   "Motor hız tepkisi zaman sabiti [s] (konum tahmini)"),
  PARAM_ROW(kyaw,        0.0f,    5.0f,  0.4f,   "Yön tutma P [%/°]"),
  PARAM_ROW(kyaw_d,      0.0f,    1.0f,  0.03f,  "Yön tutma D [%/(°/s)]"),
  PARAM_ROW(yaw_dir,    -1.0f,    1.0f,  1.0f,   "Yön tutma işareti: +1 / -1, 0 = kapalı"),
  PARAM_ROW(deadband,    0.0f,   50.0f, 12.0f,   "Motor ölü bölge telafisi [%]"),
  PARAM_ROW(max_out,    20.0f,  100.0f, 100.0f,  "En yüksek motor gücü [%]"),
  PARAM_ROW(slow_decay,  0.0f,    1.0f,  1.0f,   "1 = frenli PWM (doğrusal hız), 0 = serbest PWM"),
  PARAM_ROW(inv_l,       0.0f,    1.0f,  0.0f,   "Sol motor yönünü ters çevir (0/1)"),
  PARAM_ROW(inv_r,       0.0f,    1.0f,  0.0f,   "Sağ motor yönünü ters çevir (0/1)"),
  PARAM_ROW(pwm_hz,    100.0f, 40000.0f, 20000.0f, "PWM frekansı [Hz]"),
  PARAM_ROW(tau,         0.05f,   5.0f,  0.6f,   "Açı füzyonu zaman sabiti [s]: büyük = gyro'ya daha çok güven"),
  PARAM_ROW(d_lpf,       0.0f,  200.0f, 25.0f,   "D terimi gyro filtresi [Hz], 0 = kapalı"),
  PARAM_ROW(fall_angle, 10.0f,   80.0f, 40.0f,   "Bu eğimi geçerse düştü sayılır [°]"),
  PARAM_ROW(start_angle, 1.0f,   20.0f,  5.0f,   "Dik tutulunca başlama eşiği [°]"),
  PARAM_ROW(auto_start,  0.0f,    1.0f,  1.0f,   "1 = dik tutulunca kendiliğinden dengeye başlar"),
  PARAM_ROW(tel_div,     1.0f,   50.0f,  2.0f,   "Telemetri seyreltme: 1 = 500 Hz, 2 = 250 Hz"),
  PARAM_ROW(self_right,  0.0f,    1.0f,  1.0f,   "1 = düşünce kendi kendine kalkar"),
  PARAM_ROW(rise_back,   0.0f,  100.0f, 100.0f,  "Kalkış: önce ters yöne hamle gücü [%]"),
  PARAM_ROW(rise_back_ms, 0.0f, 1000.0f, 300.0f, "Kalkış: ters hamle süresi [ms]"),
  PARAM_ROW(rise_kick,  30.0f,  100.0f, 100.0f,  "Kalkış: yattığı yöne itme gücü [%]"),
  PARAM_ROW(rise_catch,  5.0f,   40.0f, 25.0f,   "Kalkış: bu eğimin altında denge devralır [°]"),
  PARAM_ROW(rise_tries,  0.0f,   10.0f,  3.0f,   "Kalkış: art arda en fazla deneme"),
  PARAM_ROW(vbat_div,    0.0f,   10.0f,  0.0f,   "Pil bölücü oranı (100k/100k = 2), 0 = bölücü yok"),
  PARAM_ROW(vbat_ref,    3.0f,    4.3f,  3.7f,   "Motor gücü bu pil voltajına göre ölçeklenir [V]"),
  PARAM_ROW(vbat_min,    2.8f,    3.8f,  3.3f,   "Bu voltajın altında 2 sn kalırsa dengeyi bırakır [V]"),
};

const size_t PARAM_COUNT = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

static void paramsClampAll() {
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    const ParamInfo& p = PARAM_TABLE[i];
    if (isnan(*p.value)) *p.value = p.defaultValue;
    *p.value = constrain(*p.value, p.minValue, p.maxValue);
  }
}

void paramsSetDefaults() {
  for (size_t i = 0; i < PARAM_COUNT; i++) *PARAM_TABLE[i].value = PARAM_TABLE[i].defaultValue;
}

void paramsLoad() {
  paramsSetDefaults();
  prefs.begin("denge", true);
  // Saves from an older firmware are a prefix of today's struct (fields are only ever
  // appended), so they are read as far as they go and the rest keeps its defaults.
  const size_t storedLength = prefs.getBytesLength("params");
  const bool compatible = prefs.getUInt("pver", 0) == PARAMS_VERSION && storedLength > 0 &&
                          storedLength <= sizeof(Params) && storedLength % sizeof(float) == 0;
  if (compatible) prefs.getBytes("params", &P, storedLength);
  // The learned trim is also stored on its own so it survives without a full "save".
  if (prefs.isKey("trim")) P.trim = prefs.getFloat("trim", P.trim);
  prefs.end();
  paramsClampAll();
}

void paramsSave() {
  prefs.begin("denge", false);
  prefs.putUInt("pver", PARAMS_VERSION);
  prefs.putBytes("params", &P, sizeof(Params));
  prefs.putFloat("trim", P.trim);
  prefs.end();
}

void paramsSaveTrim() {
  prefs.begin("denge", false);
  prefs.putFloat("trim", P.trim);
  prefs.end();
}

const ParamInfo* paramFind(const String& name) {
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    if (name.equalsIgnoreCase(PARAM_TABLE[i].name)) return &PARAM_TABLE[i];
  }
  return nullptr;
}

bool paramSet(const String& name, float value, String& message) {
  const ParamInfo* p = paramFind(name);
  if (!p) {
    message = "bilinmeyen parametre: " + name;
    return false;
  }
  if (isnan(value)) {
    message = "gecersiz deger: " + name;
    return false;
  }
  const float clamped = constrain(value, p->minValue, p->maxValue);
  *p->value = clamped;
  message = String(p->name) + "=" + String(clamped, 4);
  if (clamped != value) message += " (sinira cekildi)";
  return true;
}
