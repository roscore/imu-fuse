# imu-fuse

## 한국어

6축 IMU(자이로 + 가속도계)의 자세 융합 실험실입니다. "내 로봇 IMU의 자이로 잡음/바이어스와 가속도계 잡음이
이 정도이고 몸체가 이렇게 움직일 때, 어떤 자세 필터와 게인이 롤/피치 오차를 가장 작게 만들고, 요는 얼마나 빨리
드리프트하는가?"를 브라우저에서 바로 확인합니다. 모든 계산은 브라우저 안(WebAssembly)에서만 이루어지며 네트워크
요청, 분석 도구, 외부 폰트/CDN이 없습니다.

### 모델과 가정
- 실제 자세(단위 쿼터니언), 몸체 각속도, 선형 가속도를 시나리오(정지 / 느린 흔들림 / 걷는 휴머노이드 몸통 /
  빠른 회전)로 합성합니다.
- 자이로 = ω + 바이어스(t) + 백색 잡음, 가속도 = Rᵀ(a − g) + 바이어스 + 백색 잡음. 바이어스는 상수 + 랜덤워크.
  고정 시드 PRNG(Box-Muller)로 재현 가능합니다.
- 필터: 상보 필터(시정수 τ), Mahony(Kp, Ki, 자이로 바이어스 추정), Madgwick(β). 모두 지수 사상으로 쿼터니언을
  정확히 적분하고 정규화합니다.
- 지표: RMS/최대 롤·피치 오차, 끝 시점 요 드리프트, 30° 초기 오차에서 2° 미만으로 수렴하는 시간, Mahony
  바이어스 추정값. 자동 튜닝은 각 필터의 주 게인을 로그 간격 20개 값으로 훑습니다.
- 합성 신호만 사용합니다. 지자기계 없음(요는 관측 불가), 백색 잡음 + 바이어스 랜덤워크만, 온도 효과 없음.
  프리셋은 "일반적" 예시 값이며 특정 제품 데이터가 아닙니다.

### 조작
- 3D 뷰: 드래그 회전, 오른쪽 드래그/두 손가락 이동, 휠/핀치 확대, 더블클릭 초기화. 원근/위/옆/몸통 시점.
- "정지" 시나리오에서는 몸통을 끌어서 기울이면 시뮬레이션이 즉시 다시 실행됩니다.
- 재생/일시정지, 0.1×/0.25×/1× 속도, 시간 슬라이더. 추정 고스트, 몸체 축, 가속도 화살표, 기울기 원뿔 토글.

### 로컬 빌드
```sh
cargo test
trunk build --release   # dist/
sh tools/privacy_check.sh
```

라이선스: MIT. **교육용 추정치입니다 — 실제 하드웨어로 검증하세요.**

## English

An attitude-fusion lab for 6-axis IMUs (gyro + accelerometer): "My robot's IMU has this much gyro noise/bias and
accelerometer noise, and the body moves like this — which attitude filter and which gains give the smallest
roll/pitch error, and how fast does yaw drift?" Everything runs in the browser (WebAssembly); no network requests,
analytics, external fonts or CDNs.

### Model and assumptions
- Ground-truth attitude (unit quaternion), body angular velocity and linear acceleration are synthesised for a
  scenario (static / slow sway / walking humanoid torso / fast spin).
- gyro = ω + bias(t) + white noise; accel = Rᵀ(a − g) + bias + white noise. Bias = constant + random walk.
  Seeded PRNG (Box-Muller), reproducible.
- Filters: complementary (time constant τ), Mahony (Kp, Ki with gyro-bias estimate), Madgwick (β). All integrate
  quaternions exactly with the exponential map and renormalise.
- Metrics: RMS / max roll-pitch error, yaw drift at the end, settling time from a 30° initial tilt error to below
  2°, Mahony bias estimate. Auto-tune sweeps each filter's main gain over 20 log-spaced values.
- Synthetic signals only. No magnetometer (yaw is unobservable), white noise + bias random walk only, no
  temperature effects. Presets are "typical" illustrative values, not any product's datasheet.

### Controls
- 3D view: drag to orbit, right-drag / two fingers to pan, wheel / pinch to zoom, double-click to reset.
  Perspective / Top / Side / Body views.
- In the "static" scenario, drag the torso to tilt it; the simulation re-runs live.
- Play / pause, 0.1× / 0.25× / 1× speed, time slider. Toggles for ghost estimates, body axes, accel arrow, tilt cone.

### Local build
```sh
cargo test
trunk build --release   # dist/
sh tools/privacy_check.sh
```

License: MIT. **Educational estimates — validate on real hardware.**
