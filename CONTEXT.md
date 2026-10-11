# Mobin

Camera와 LiDAR로 사람을 관측하고, 이동 상태를 추정하여 robot의 안전한 이동을 판단하는 mobile robot 실습 context다.

## Language

**Camera–LiDAR Association**:
Camera의 person bbox와 LiDAR point 후보를 연결하여 사람의 3D 관측 위치를 얻는 과정이다. Bbox 안에 투영되었다는 사실만으로 해당 point가 실제 사람에 속한다고 확정하지 않는다.
_Avoid_: 3D association, sensor fusion

**Instance Label Map**:
한 Camera frame에서 각 pixel이 배경인지 어느 person instance에 속하는지를 `mono8` 값 `0..N`으로 나타내는 image다. 이 값은 frame-local label이며 지속적인 Person Track ID가 아니다.
_Avoid_: Track ID Image, mono16 mask

**Camera Visibility**:
LiDAR point가 나타내는 표면이 Camera 시점에서 다른 표면에 가려지지 않고 보이는 관계다. Image 안으로 투영되거나 person label과 겹친다는 사실만으로 이 관계가 성립하지는 않는다.
_Avoid_: Camera FoV, person classification

**Ground Reference Frame**:
주행 가능한 평면을 `xy`, 그 평면으로부터의 높이를 `z`로 해석하기 위한 robot-local 좌표계다. 현재 TurtleBot3 모델에서는 `base_footprint`를 사용한다.
_Avoid_: LiDAR frame, Ground Frame

**Foreground Depth Candidate**:
Person bbox 안으로 투영된 LiDAR point 중 Camera 깊이축에서 하나의 물체일 가능성이 있는 point 집합이다. Camera에 가장 가깝다는 사실만으로 사람이라고 확정되지 않는다.
_Avoid_: Person Depth Cluster, foreground person

**Ambiguous Association**:
하나의 person 영역에서 둘 이상의 Foreground Depth Candidate가 비슷한 근거를 가져 고유한 Person Observation을 정할 수 없는 상태다. 이 상태에서는 임의의 후보를 선택하지 않는다.
_Avoid_: Nearest candidate, uncertain person

**Person Observation**:
한 timestamp에서 Camera–LiDAR Association으로 얻은 사람의 3D 위치 측정이다. 지속적인 ID나 속도를 포함하지 않는다.
_Avoid_: Person Track, detection result

**Camera–LiDAR Observation Time Gap**:
하나의 Camera–LiDAR Association에 사용한 Camera와 LiDAR의 관측 시각 차이다. 두 관측 시각은 비교 가능한 공통 시간 기준으로 표현되어야 하며, YOLO 결과가 도착하기까지 걸린 시간과 구분한다.
_Avoid_: Inference latency, arrival time gap

**Person Observation Age**:
판단 시각에서 Person Observation의 관측 시각을 뺀 경과 시간이다. 관측 결과를 최근에 수신했더라도 원래 관측이 오래되었다면 이 값은 크다.
_Avoid_: Receipt age, sensor synchronization error

**Person Track**:
`odom` 좌표계에서 ID, 위치, 속도와 위치 불확실성을 함께 나타내는 사람 상태다.
_Avoid_: Detection, tracking result

**Track Assignment**:
현재 Person Observation과 기존 Person Track을 일대일로 연결하여 여러 frame에서 사람 ID를 유지하는 과정이다. 물체의 semantic class나 LiDAR point의 소속을 판정하지 않는다.
_Avoid_: Hungarian tracking, semantic association

**Pallet Jack**:
`crossing`에서 사람과 함께 움직이는 수동 운반 장비다. Person Track 대상이 아니며 `tracker` 또는 일반적인 `cart`로 부르지 않는다.
_Avoid_: Tracker, cart

**CPA Risk**:
`odom` 좌표계에서 Robot과 Person이 현재 속도를 유지할 때의 최근접 시간과 최근접 거리에 근거한 미래 근접 위험이다.
_Avoid_: CPA point, collision point, TTC

**LiDAR Emergency**:
Robot의 요청 이동 방향에서 현재 LiDAR 최소거리가 안전 threshold 이하이거나 유효한 거리 입력이 없을 때 발생하는 즉시 정지 위험이다. 물체가 갑자기 나타났는지는 판단하지 않는다.
_Avoid_: Sudden appearance, CPA Risk

**Command Arbitration**:
Navigation과 teleoperation처럼 동시에 존재할 수 있는 Robot 속도 요청 중 유효하고 우선순위가 높은 하나를 안전 판단 전에 선택하는 과정이다.
_Avoid_: Safety Intervention, velocity control

**Safety Intervention**:
예측 충돌 위험, LiDAR 근거리 위험 또는 입력 만료를 근거로 요청된 robot 속도를 감속하거나 정지시키는 행위다.
_Avoid_: Obstacle avoidance, emergency

**Closed-loop Verification**:
실제 sensor topic 입력부터 최종 robot 속도 명령의 변화까지 하나의 실행에서 이어졌음을 확인하는 검증이다.
_Avoid_: Unit test, topic smoke test, 구현 완료

**Tracking True Positive (TP)**:
Camera–LiDAR 공통 ROI의 실제 사람과 Person Track이 같은 timestamp에서 평가 거리 이내로 일대일 연결된 결과다.
_Avoid_: YOLO detection, association success

**Tracking False Negative (FN)**:
Camera–LiDAR 공통 ROI에 실제 사람이 있지만 평가 거리 이내로 연결할 Person Track이 없는 누락 결과다.
_Avoid_: True Negative, sensor ROI exclusion

**Tracking False Positive (FP)**:
Camera–LiDAR 공통 ROI에 Person Track이 있지만 평가 거리 이내로 연결되는 실제 사람이 없는 결과다. 실제 사람이 전혀 없다는 뜻으로 한정하지 않으며 중복 Track과 큰 위치 오차도 포함한다.
_Avoid_: YOLO false positive, collision

**Identity F1 (IDF1)**:
전체 평가 시간에서 실제 사람 ID와 Person Track ID가 일관되게 대응한 정도를 precision과 recall의 조화평균으로 나타낸 값이다.
_Avoid_: Detection F1, ID switch count

**Trial**:
하나의 고정된 환경과 시나리오에서 수행하고, 실행 조건과 결과 증거를 `trials/<run-name>/`에 함께 보존한 검증 실행이다.
_Avoid_: Run, demo, test result

**Synthetic Sensor Case**:
사람 영역과 센서 측정값을 직접 구성하여 특정 입력 조건에서 노드의 출력과 알려진 실패를 확인하는 사례다. Gazebo 센서 생성이나 인식 및 추적 전체의 성능 평가와 구분한다.
_Avoid_: Gazebo trial, real-world validation, detection success rate

**Fixed-input Replay Evaluation**:
같은 저장 입력과 정답에 변경 전후의 판단 조건을 적용하여 해당 변경의 영향을 비교하는 평가다. 입력이나 실행 조건이 다른 별도 Trial끼리의 비교와 구분한다.
_Avoid_: Separate-run comparison, full-system improvement

**regression-test**:
기존 계산과 상태 판단이 코드 변경 후에도 유지되는지 자동으로 확인하는 Trial이다.
_Avoid_: Unit demo

**crossing**:
사람과 Pallet Jack이 고정된 시작 조건과 경로로 robot 진행 방향을 가로지르는 Gazebo 상황이다. 특정 회피 알고리즘의 성공을 뜻하지 않는다.
_Avoid_: CPA test, avoidance result

**deterministic_crossing**:
`crossing` 상황에 Camera–LiDAR Association, Person Track과 CPA 기반 Safety Intervention을 연결하여 안전 정지와 복귀를 반복 확인하는 Trial이다.
_Avoid_: Random crossing, obstacle demo

**seeded-multi-person-tracking**:
고정된 random seed로 여러 사람을 움직여 association과 Person Track의 일관성을 평가하는 Trial이다.
_Avoid_: Multi-person avoidance

**safety-test**:
입력 만료, 근거리 위험과 node 중단을 의도적으로 만들어 안전 제한과 watchdog 정지를 확인하는 Trial이다.
_Avoid_: Safety demo, fault-free run
