# BOSIO 출력 코어
## 영상으로 함 보시죠
[![이미지 텍스트](http://i.ytimg.com/vi/5v-ZqBwbi3k/0.jpg)](https://www.youtube.com/watch?v=5v-ZqBwbi3k)
유튜브 링크입니다.

## 들어가기 전에,
안녕하세요. 저는 VARZero 계정주인입니다.  
제가 생각하던 정이십면체 프레임버퍼 시스템이 있었는데요, 이를 출력하는 하드웨어 코어를 구현하려 했습니다.  
근데 이왕이면 요새 유행하는 에이전틱 AI를 이용해서 코드를 작성해볼까?로 시작했는데요..  
그러다보니까 만약 내가 생각한것만 던져줘도 얘가 완성할까? 라는 생각으로  
정이십면체 프레임버퍼 개념, 타일 분할, 초기 중앙 면 추적 시스템(이름 참 이상하네요)을 정의해 두고, 얘네한테 다 시켜봤습니다..

사용했던 것들은  
Antigravity(Gemini Flash 3.7, 3.8)으로 초기 제작을 진행하다가  
Codex(GPT-6 Astra, GPT-5.6 Sol/Luna)를 이용하여 완성해봤습니다.

아래 있는 모든 내용은 전부 LLM이 작성했습니다.  
한줄 코멘트를 달면, "미쳤네요. 얘네 하드웨어도 다 만들수 있을것 같은데요?"

참고로 보시오 프로젝트는 정이십면체 프레임버퍼를 이용해서  
3DoF용 HMD 디스플레잉 시스템을 만드는 것이고,  
이프로젝트는 ~~제가~~ LLM이 HDMI쪽으로 제 아이디어의 최종 출력단을 만들어봤습니다.  
이 다음은 이 시스템에 맞게 투영하는 코어를 "을숙도 아키텍쳐"로 완성하는것입니다.

아무쪼록 관심있게 봐주시면 감사하겠습니다. (꾸벅)

## 아래부터 본문.
# BOSIO 출력 코어

정이십면체 기반 삼각형 프레임버퍼를 위한 AXI 기반 RTL 출력 코어입니다.
출력 픽셀마다 정이십면체상의 방향을 계산하고, 해당하는 면·7-7-7-8 타일·
삼각형 셀을 선택한 뒤, 장면 캐시에서 색상을 읽어 RGB 영상으로 출력합니다.

raw 3DoF 자세 스트림도 지원합니다. 센서 모드에서는 yaw·pitch·roll을
FPGA 내부에서 180개의 affine Q24 투영 계수로 변환합니다. PYNQ에서 코어를
제어하기 위한 최소 Python 드라이버와 장면 패킹 모듈도 함께 제공합니다.

이 디렉토리는 독립적인 Vivado IP 저장소로 사용할 수 있도록 구성되어
있습니다.

이 코어를 사용하는 구면 윈도우 데몬, C++/ARM NEON dirty tile 합성기,
GY-521 센서 허브와 PYNQ-Z2 통합 빌드는
[VARZero/bosio_SphericalWM](https://github.com/VARZero/bosio_SphericalWM)에서
관리합니다.

기여자 및 AI 개발 지원 내역은 [CONTRIBUTORS.md](CONTRIBUTORS.md)에서
확인할 수 있습니다.

## 공개 범위

기본 빌드는 BS25 파라미터형 DDR 읽기 캐시를 사용합니다. 셀 색 데이터는
DDR에 유지하고, 기본 64바이트 라인·16KiB·2-way 캐시로 읽습니다.
[BS25 캐시 문서](docs/CACHE_LINE.md)에 파라미터, 새 레지스터, DDR 버퍼 수명과
부분 갱신 방법을 설명했습니다. 아래의 기존 BRAM 용량·BPT1 직접 DMA 설명은
`DDR_CACHE_ENABLE=0`인 BS24에 해당합니다. BS25에서는 제공 드라이버가 BPT1을
DDR 장면에 적용합니다.

포함 항목:

- `component.xml`: Vivado IP-XACT 패키지 메타데이터
- `hdl/`: 합성 가능한 Verilog RTL
- `xgui/`: Vivado IP 설정 GUI 메타데이터
- `software/`: PYNQ 드라이버와 장면/기하 패킹 모듈
- `README.md`: 본 문서

전체 PYNQ 서비스, 창 GUI, 보드 블록 디자인, HDMI 전기적 출력 래퍼,
센서 에뮬레이터는 포함하지 않습니다. 외부 시스템이 DDR, AXI 인터커넥트,
영상 타이밍, HDMI/DVI 송신기를 제공해야 합니다.

`software/`에는 코어 제어에 필요한 최소 PYNQ 드라이버와 정이십면체 장면
패킹 모듈이 들어 있습니다. 서비스 데몬이나 창 GUI 없이도 bitstream을
로드하고, 장면을 업로드하고, 수동 자세 또는 센서 모드를 선택할 수 있습니다.

```text
software/
  bosio_driver_v2.py       # PYNQ Overlay·DMA·레지스터 제어
  bosio_geometry_v2.py     # 정이십면체 좌표·Q24·장면 패킹
  requirements.txt          # numpy, pynq
  README.md                 # 드라이버 사용법
```

## 구조

```text
                   +------------------------------+
AXI4-Lite -------->| 제어 및 계수 레지스터 인터페이스 |
                   +---------------+--------------+
                                   |
AXI4-Stream sensor -> mrad 자세 -> 투영 계수 엔진
                                   |
                                   v
                         180개 shadow 계수 테이블
                                   |
                                   v
화면 x/y -> barycentric DDA -> 정규화 -> 타일/셀 주소
                                   |
DDR 장면 --AXI read-- 듀얼 BRAM 캐시 -> 팔레트 -> 경계 AA -> RGB FIFO
                                                           |
                                                           v
                                             AXI4-Stream RGB 영상
```

현재 최상위 RTL의 출력 해상도는 1280 × 720입니다. `resolution` 레지스터로
삼각형 셀 분할도를 `M=8`, `M=16`, `M=32` 중에서 선택합니다. 영상 타이밍
생성기는 이 코어 외부에 있습니다.

## 정이십면체 배열

정이십면체의 각 20개 면은 211개의 삼각형 타일을 가집니다.

```text
7×7 + 7×7 + 7×7 + 8×8 = 면당 211개 타일
```

각 타일은 `M×M` 삼각형 셀로 구성됩니다. 셀 인덱스는 행 우선이며 방향을
포함합니다.

```text
cell = row² + 2·column + orientation
```

행의 마지막 셀은 하나의 방향만 가집니다. 이 표현은 정사각형 UV 텍스처를
사용하지 않고 삼각형 경계의 셀 중복을 피합니다.

## 장면 메모리 형식

AXI 마스터는 외부 DDR의 패킹된 장면을 32비트 데이터와 16-beat 증가 버스트로
읽습니다. 장면 시작 주소는 64바이트 정렬이어야 합니다.

| 워드 범위 | 내용 |
|---:|---|
| `0 .. 255` | 256-entry RGB24 팔레트 |
| `256 .. 4475` | 20 × 211 타일 디렉터리 |
| `4476 ..` | 패킹된 8비트 셀 색상 인덱스 |

디렉터리 항목은 셀 데이터 영역의 바이트 오프셋입니다. `0xffffffff`는
비활성 타일을 의미합니다. 캐시 용량은 bank당 196,608바이트이며, 헤더를
포함한 최대 장면 전송 크기는 53,632워드입니다.

새 장면은 현재 사용하지 않는 BRAM bank로 수신한 뒤 프레임 경계에서 교체됩니다.
교체 후 같은 장면을 이전 bank에도 복제하므로 두 bank는 다음 부분 갱신의 동일한
기준 장면을 유지합니다. `BPT1` 패치는 기존 directory offset을 사용하는 타일만
비활성 bank에 기록하고 프레임 경계에서 교체한 다음 이전 bank에도 재적용합니다.
따라서 전체 장면과 부분 갱신 모두 화면 중간에 바뀌지 않습니다.

부분 갱신 패킷은 16워드 정렬 형식입니다. 파일 헤더 16워드 뒤에 타일마다
16워드 record header와 `M*M/4`개의 payload word가 옵니다. 파일 헤더의 word
0은 `0x42505431`(`BPT1`), word 1은 타일당 payload word 수, word 2는 record 수,
word 3은 전체 word 수입니다. record header의 word 0은 셀 데이터 RAM의 word
offset이고 word 1은 진단용 global tile ID입니다. 타일 활성 여부나 directory
배치가 달라질 때는 전체 장면을 다시 올려야 합니다.

RGB 장면을 이 형식으로 만드는 방법은 이 코어가 정의하지 않습니다. 외부
장면 생성기가 팔레트·디렉터리·셀 데이터를 생성해야 합니다.

## 인터페이스

### 클록과 리셋

- `aclk`: 모든 인터페이스가 사용하는 동기 클록
- `aresetn`: active-low 리셋

참조 PYNQ-Z2 설계는 100 MHz를 사용합니다. 클록 생성은 외부 시스템이 담당합니다.

### AXI4-Lite 슬레이브

기본 인터페이스는 32비트 데이터와 7비트 byte address입니다. 참조 소프트웨어는
full byte strobe로 레지스터를 기록합니다.

### AXI4 Full 마스터

기본 인터페이스는 32비트 읽기 전용 마스터입니다. 코어는 16-beat 증가 버스트를
발행합니다(`ARLEN=15`, `ARSIZE=2`). `RRESP`, `RVALID`, `RREADY`, `RLAST`를
검사합니다.

### 센서 AXI4-Stream 슬레이브

한 번의 전송에 자세 샘플 하나가 들어갑니다.

| 비트 | 형식 | 의미 |
|---:|---|---|
| `31:0` | signed `int32` | yaw, milliradian |
| `63:32` | signed `int32` | pitch, milliradian |
| `95:64` | signed `int32` | roll, milliradian |

참조 구현은 별도 timestamp 없이 96비트 TDATA를 사용합니다. 센서 송신기는
TVALID 동안 TDATA를 유지하고 `TVALID && TREADY`에서 샘플을 전달해야 합니다.

### 영상 AXI4-Stream 마스터

- `TDATA`: RGB24 (`VIDEO_TDATA_WIDTH=24`)
- `TVALID/TREADY`: 표준 스트림 핸드셰이크
- `TUSER`: 프레임 첫 픽셀에서 asserted (SOF)
- `TLAST`: 각 라인의 마지막 픽셀에서 asserted (EOL)

이 스트림은 `v_axi4s_vid_out` 같은 외부 영상 타이밍/포맷 블록에 연결한 뒤,
보드별 HDMI/DVI 송신기에 연결합니다.

## 투영 계수 형식

투영기는 60행 × 3개의 signed Q24 값을 사용합니다.

```text
row[face] = { start, dx, dy }
Q24 value = real_value × 2²⁴
```

각 화면 픽셀에서 세 값이 한 면의 barycentric numerator를 affine 방식으로
생성합니다. 화면을 가로·세로로 훑는 동안 DDA가 덧셈만으로 값을 갱신합니다.

180개 계수는 shadow table에 먼저 기록됩니다. commit이 들어오면 다음 프레임
시작 시점에 전체 계수가 원자적으로 적용되므로 한 프레임 안에서 서로 다른
자세의 계수가 섞이지 않습니다.

## 센서 자세 엔진

`hdl/bosio_v2_sensor_pose.v`는 다음을 FPGA 내부에서 수행합니다.

1. raw mrad 패킷을 저장하고 처리 중 들어온 패킷은 최신값 기준으로 합칩니다.
2. 3개의 20-iteration 원형 CORDIC로 sine/cosine을 계산합니다.
3. Q2.30 카메라 중심·right·up basis를 생성합니다.
4. 20개 정이십면체 면 역행렬과 곱셈합니다.
5. 60개의 `(start, dx, dy)` 행을 signed Q24로 생성합니다.
6. 계수를 projector shadow table에 기록하고 원자적 commit을 요청합니다.

현재 센서 모드의 투영 FOV는 1280 × 720 기준 가로 60°, 세로 45°로 고정되어
있습니다. 100 MHz에서 측정한 변환 지연은 628클록, 즉 6.28 µs입니다.

연속 센서 스트림에서는 다음 패킷이 곧바로 처리되므로 busy 상태가 대부분
true로 보일 수 있습니다. 소프트웨어는 busy가 잠깐 false가 되는 순간보다
적용된 packet ID의 증가를 완료 조건으로 사용해야 합니다.

## AXI-Lite 레지스터

주소는 byte address입니다.

| 주소 | R/W | 설명 |
|---:|:---:|---|
| `0x00` | R/W | bit 0: 출력 enable |
| `0x04` | R | bit 0 enabled, bit 1 scene valid, bit 2 DMA busy, bit 3 pose pending, bit 4 scene pending, bit 5 error, bit 31:16 frame counter |
| `0x08` | R/W | DDR 장면 base address |
| `0x0c` | R/W | 장면 word count |
| `0x10` / `0x14` / `0x18` | R | BS25 cache hit / miss / 요청 대기 클럭 |
| `0x1c` | R/W | AA 설정: bit 0 enable, bit 15:8 경계 임계값, bit 23:16 혼합 강도 |
| `0x20` | R/W | 센서 축 반전: bit 0 yaw, bit 1 pitch, bit 2 roll |
| `0x24` | R | raw yaw mrad, signed 32비트 |
| `0x28` | R | raw pitch mrad, signed 32비트 |
| `0x2c` | R | raw roll mrad, signed 32비트 |
| `0x30` | R | 수신 센서 packet counter |
| `0x34` / `0x38` / `0x3c` | R | BS25 라인 바이트 / 캐시 바이트 / way 수 |
| `0x40` | R | BS25 bit 0: 픽셀 요청·응답 배출 완료 |
| `0x44` | R | BS25 활성 DDR 장면 BASE |
| `0x5c` | R/W | resolution: 0=`M8`, 1=`M16`, 2=`M32` |
| `0x60` | R/W | 수동 계수 write index |
| `0x64` | W | 수동 계수 data; 기록 후 index 증가 |
| `0x68` | W | bit 0: 수동 계수 commit |
| `0x6c` | W | bit 0: 전체 장면 로드, bit 1: BS25 DDR 주소 전환 / BS24 BPT1 직접 적용 |
| `0x70` | R | 수신 word count. BS25는 메타데이터, BS24는 전체 DMA |
| `0x74` | R | BS25 최대 셀 데이터 4,321,280바이트 / BS24 BRAM 데이터 196,608바이트 |
| `0x78` | R/W | write bit 0: 센서 모드 enable; read bit 1 sensor active, bit 2 pose engine busy, bit 31:16 마지막 적용 packet ID 하위 16비트 |
| `0x7c` | R | 기본 `0x42533235` (BS25) / legacy `0x42533234` (BS24) |

센서 모드가 켜져 있으면 수동 계수 commit은 무시됩니다. 수동 자세를
설정하려면 먼저 센서 모드를 끄십시오.

## RTL 파일

| 파일 | 역할 |
|---|---|
| `bosio_output_top.v` | 최상위 인터페이스, 레지스터 ABI, 파이프라인 연결 |
| `bosio_v2_sensor_pose.v` | raw mrad 자세 변환 및 CORDIC |
| `bosio_v2_projector.v` | 면 선택 및 barycentric affine DDA |
| `bosio_v2_cache.v` | DDR 리더, sparse directory, palette, 듀얼 BRAM 캐시 |
| `bosio_v2_edge_aa.v` | 한 줄 버퍼 기반 경계 적응형 투영 AA |
| `bosio_out_fifo.v` | RGB24 출력 FIFO |
| `bosio_out_stream_out.v` | 영상 AXI4-Stream 프레이밍 |
| `bosio_out_sensor_rx.v` | 센서 패킷 수신기 |
| `bosio_out_agu.v` | 이전 버전 호환/지원 주소 생성 로직 |
| `bosio_out_bary_agu.v` | 이전 버전 호환/지원 barycentric 주소 로직 |
| `bosio_out_pose_proc.v` | 이전 버전 호환/지원 자세 처리 로직 |
| `bosio_out_reg_ctrl.v` | 이전 버전 호환/지원 레지스터 제어 로직 |
| `bosio_out_dma_cache.v` | 이전 버전 호환/지원 DMA 캐시 로직 |

v2 최상위는 `bosio_v2_*` 파이프라인과 센서 수신기를 사용합니다. 나머지
`bosio_out_*` 파일은 패키지의 소스 집합에 포함되어 있으며 이전 통합 환경과의
호환성을 위해 보존되어 있습니다.

## Vivado 패키징

Vivado에서 이 디렉토리를 사용자 IP 저장소로 추가하거나 다음과 같이 직접
패키징할 수 있습니다.

```tcl
create_project -in_memory -part xc7z020clg400-1
add_files [glob ./hdl/*.v]
set_property top bosio_output_top [current_fileset]
update_compile_order -fileset sources_1
ipx::package_project -root_dir [file normalize .] \
    -vendor varzero.org -library user -taxonomy /Display \
    -import_files -force
```

패키징 후 Vivado IP Settings에서 저장소 경로를 추가하고,
`varzero.org:user:bosio_output_core:1.0`을 IP Integrator에 인스턴스화합니다.
`component.xml`에는 AXI-Lite, AXI Full master, 센서 AXI4-Stream, 영상
AXI4-Stream, clock, reset 연결 정보가 포함되어 있습니다.

## 통합 체크리스트

1. 목표 타이밍을 만족하는 클록·active-low 리셋을 연결합니다.
2. AXI-Lite 슬레이브를 프로세서 또는 제어 마스터에 연결합니다.
3. AXI Full master를 DDR 및 AXI 인터커넥트에 연결하고 scene base를 64바이트
   정렬합니다.
4. 위에서 정의한 메모리 형식으로 장면을 준비합니다.
5. 센서 모드가 필요하면 센서 AXI4-Stream을 연결합니다.
6. 영상 스트림을 외부 타이밍 및 HDMI/DVI 경로에 연결합니다.
7. 장면을 업로드하고 수동 계수 또는 센서 모드를 설정한 뒤 enable을 켭니다.
8. 초기화 시 `0x04`, `0x30`, `0x70`, `0x78`을 모니터링합니다.

## BS25 참조 검증 결과

64바이트 라인·16KiB·2-way를 사용하는 PYNQ-Z2 통합 빌드는 100MHz에서
LUT 23,733개(44.61%), 레지스터 24,821개, BRAM 21.5/140개(15.36%), DSP 83개를
사용했습니다. 최종 WNS는 +0.111ns, WHS는 +0.019ns입니다.
기존 BS24 참조 빌드의 BRAM 사용량은 아래 표의 137.5/140개였습니다.

실제 보드에서 M=16 전체 구면 4220타일(1,098,240바이트 장면)을 읽으며
약 59.9FPS를 확인했습니다. 이 테스트의 cache hit 비율은 99.86%였으며,
한 타일의 부분 갱신 8회를 두 DDR 버퍼에 적용하고 주소 전환을 확인했습니다.
M=32는 전체 구면 패킹만 확인했으며, 실시간 출력 성능을 측정한 결과는 아닙니다.
원본 수치와 보고서는 통합 저장소의 `verification/results/`에 있습니다.

## BS24 참조 검증 결과

PYNQ-Z2 참조 빌드는 100 MHz에서 다음 결과를 얻었습니다.

| 자원 | 사용량 |
|---|---:|
| LUT (logic + distributed memory) | 26,894 (50.55%) |
| Register | 24,710 (23.22%) |
| BRAM tile | 137.5 / 140 (98.21%) |
| DSP | 83 (37.73%) |
| WNS / WHS | +0.281 ns / +0.051 ns |

921,600픽셀 AXI/backpressure 시스템 테스트, 5개 자세의 raw sensor 계수
테스트, 전체 장면 이후 부분 패치를 두 cache bank에 적용하는 RTL 테스트와
AA 픽셀 개수·순서·혼합값 테스트를 통과했습니다. 위 수치는 부분 타일 갱신과
경계 AA RTL을 포함한 참조 Zynq-7020 구현 결과이며, 다른 FPGA나 파라미터를
사용할 때는 다시 합성·검증해야 합니다.

부분 갱신 회귀 테스트는 `verification/tb_partial_tile_cache.v`, AA 회귀 테스트는
`verification/tb_edge_aa.v`에 있습니다. 두 테스트는 통합 저장소인
`bosio_SphericalWM`에서 제공합니다.

## 알려진 제한사항

- 현재 최상위 영상 출력은 1280 × 720으로 고정되어 있습니다.
- 센서 모드 FOV는 60° × 45°로 고정되어 있습니다.
- 장면 색상은 RGB332 인덱스와 256-entry 팔레트를 사용합니다.
- 이 코어는 장면 샘플러·투영기이며, 글꼴·창·일반 2D 도형·카메라 프레임을
  내부에서 렌더링하지 않습니다.
- BS24는 BRAM 사용량이 Zynq-7020 한계에 가깝습니다. BS25는 셀 데이터를
  DDR로 옮겼지만, 더 큰 `M`의 합성 비용과 DDR 읽기 성능은 별도 확인이 필요합니다.
- 한 줄 인과 AA 필터는 대칭 3×3 필터보다 단순하며 고주파 무늬에도 반응할 수
  있습니다. 기본 설정은 enable, threshold 24, strength 32입니다.

## 버전 관리

현재 패키지 버전은 `1.0`, core revision은 `26`, 기본 runtime signature는
BS25 `0x42533235`입니다. `DDR_CACHE_ENABLE=0`은 BS24 `0x42533234`입니다.
레지스터 ABI, 장면 메모리 형식, 센서 패킷 형식을
변경할 때는 패키지 revision을 올리고 이 README에 변경 내용을 기록하십시오.
