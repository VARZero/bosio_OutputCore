# PYNQ 드라이버

이 디렉토리는 BOSIO 출력 코어를 PYNQ에서 제어하는 최소 Python 모듈을
포함합니다.

## 파일

- `bosio_driver_v2.py`: bitstream 로드, 장면 DMA 업로드, 수동 자세 설정,
  센서 모드 전환, 상태 조회
- `bosio_geometry_v2.py`: 정이십면체 기준 좌표, 셀/타일 매핑, 수동 Q24
  투영 계수 계산, 장면 메모리 패킹

Python 3, NumPy, PYNQ가 필요합니다. 두 파일은 같은 디렉토리에 두거나
이 디렉토리를 `PYTHONPATH`에 추가해야 합니다.

## 기본 사용

```python
import numpy as np
from bosio_driver_v2 import BosioV2

driver = BosioV2("bosio_v2.bit", m=32)

# scene_rgb의 shape은 (20, 211, M*M, 3), dtype은 uint8
driver.upload(scene_rgb)
driver.set_pose(0.0, 0.0, 0.0, 60.0, 45.0)
driver.start()

# 투영 경계 AA: enable, 밝기 임계값, 혼합 강도(각 0..255)
driver.set_antialias(True, threshold=24, strength=32)

print(driver.status())
driver.close()
```

`set_pose()`의 yaw·pitch·roll은 degrees입니다. 이 함수는 Python에서
180개의 signed Q24 투영 계수를 만든 뒤 AXI-Lite로 전송합니다.

## 센서 모드

센서 AXI4-Stream이 코어에 연결되어 있다면 다음과 같이 FPGA 내부 자세
변환을 활성화할 수 있습니다.

```python
driver.use_sensor(True)
driver.set_sensor_invert(yaw=False, pitch=True, roll=True)
print(driver.status())
```

`set_sensor_invert()`는 케이스 장착 방향에 맞춰 각 축의 부호를 독립적으로
뒤집습니다. 설정은 출력 코어의 `0x20` 레지스터에 기록됩니다.

센서 스트림의 세 값은 signed `int32` milliradian이며, 패킷 형식은 다음과
같습니다.

```text
TDATA[31:0]   = yaw_mrad
TDATA[63:32]  = pitch_mrad
TDATA[95:64]  = roll_mrad
```

센서 모드에서는 `set_pose()`를 호출하지 않아도 됩니다. `status()`의
`sensor_applied` packet ID가 증가하면 FPGA가 해당 센서 샘플의 계수 적용을
완료한 것입니다.

## 장면 패킹

`pack_scene(rgb, m)`은 셀 RGB 배열을 코어가 읽는 DDR 장면으로 변환합니다.

```python
from bosio_geometry_v2 import pack_scene

words, active_tiles = pack_scene(scene_rgb, m=32)
driver.upload_words(words)
```

장면은 256-entry RGB332 팔레트, 20×211 디렉터리, 활성 타일의 packed
8-bit 셀 인덱스 순서로 구성됩니다. 장면 생성기가 창·텍스트·도형을 어떤
방식으로 셀에 그릴지는 이 드라이버의 범위가 아닙니다.

## 코어 호환성

드라이버는 bitstream signature `0x42533234`를 확인합니다. `upload_patch()`는
윈도우 합성기가 만든 `BPT1` 타일 패킷을 양쪽 BRAM bank에 원자적으로 적용합니다.
다른 core
revision이나 다른 레지스터 ABI를 사용할 때는 드라이버의 signature 검사와
레지스터 정의를 함께 갱신해야 합니다.

이 드라이버는 PYNQ-Z2 참조 설계의 `output_core_0` 인스턴스와 100 MHz
FCLK0를 기준으로 작성되었습니다. 다른 보드에서는 Overlay 계층 이름,
클록 설정, 물리 주소와 DMA 연결을 확인해야 합니다.
