PSDF ROS 로컬 플래너/서비스 점검 체크리스트

범위: `src/psdf_local_planner.cpp`, `scripts/psdf_ros_node.py`, `scripts/mpc_optimizer.py` 및 ROS 통합 상태 점검. [fix] 표시는 즉시 적용 권장 수정 사항이며, 나머지는 확인/선택 사항입니다.

로컬 플래너(C++)
- 서비스 네임스페이스: [fix] 서비스는 네임스페이스 없이 절대 경로 `"/psdf_mpc"` 로 고정합니다. 노드를 어떤 ns(예: `robot`) 아래에서 실행하더라도 서비스는 전역 `/psdf_mpc` 로 제공되도록 유지하세요. 테스트/런치 파일에서 불필요한 ns 부여(`robot`)는 제거합니다.
- 파라미터 범위: 플래너는 `~/<name>`(개별 플러그인 네임스페이스) 에서 파라미터를 읽습니다. 현재 YAML은 `robot` 네임스페이스에만 로드됩니다. `move_base/PSDFLocalPlanner` 아래에 플래너 전용 파라미터를 로드하거나, 코드 기본값에 의존하세요. `goal_tolerance_xy`, `goal_tolerance_yaw`, `psdf_mpc_service`, `service_timeout` 추가를 고려하세요.
- C++ 표준: [fix] `CMakeLists.txt` 의 `add_compile_options(-std=c++11)` 가 주석 처리되어 있습니다. 코드에 `override` 를 사용하므로 구형 ROS 배포판에서 빌드 오류를 피하려면 C++11을 활성화하세요.
- 플랜 없음 동작: `global_plan_` 이 비어있을 때 `computeVelocityCommands` 가 0 속도를 내고 `true` 를 반환합니다. `move_base` 의 복구/재플래닝을 유도하려면 `false` 반환을 고려하세요.
- 현재 속도: [fix] 로컬 플래너가 `nav_msgs/Odometry` 를 구독하여 실제 속도를 받아 서비스 요청의 `current_velocity` 를 채웁니다.
- 타임아웃 사용: `service_timeout_` 은 로드되지만 사용되지 않습니다. 논블로킹 호출 패턴에서 타임아웃을 적용하거나 호출 전 `waitForExistence(Duration(service_timeout_))` 를 고려하세요.
- 프레임/경로 헤더: `ref_path.header` 를 `current_pose` 로 설정합니다. 글로벌 플랜 프레임이 다르면 서비스가 사용하는 로컬 프레임과 일치하도록 정규화하거나, 동일 프레임 사용을 명시하세요.
- 경로 좌표계 불일치: [fix] `global_plan_` 의 각 `PoseStamped`는 보통 `map` 프레임이고, `getRobotPose()`는 로컬 코스트맵 프레임(대개 `odom`)을 반환합니다. 현재 코드는 포즈들을 변환하지 않고 `Path.poses`에 그대로 복사해 헤더만 현재 포즈의 프레임으로 덮어씁니다. `tf2`로 모든 플랜 포즈를 `current_pose.header.frame_id`(또는 서비스의 `local_frame`)로 변환한 뒤 전달하세요.
- 사소한 include: `sqrt`/`fabs` 사용을 위해 `<cmath>` 를 명시적으로 include 하는 것을 고려하세요.

파이썬 서비스 노드
- 기본 발자국(footprint): [fix] `load_params()` 기본 리스트 오타 — `[0.25 -0.25]` 에 콤마가 빠졌습니다. 4개 꼭짓점을 유지하려면 `[0.25, -0.25]` 로 수정하세요.
- 마커 프레임: [fix] 시각화 마커가 하드코딩된 `'base_link'` 를 사용합니다. 설정된 프레임과 일관성을 위해 `marker.header.frame_id` 에 `self.params['local_frame']` 를 사용하세요.
- 좌표계 일관성: [fix] 장애물(EdgeClusters) 좌표계와 상태/경로 좌표계의 일치가 보장되지 않습니다. `edge_cb`에는 프레임 확인/로그를 추가하고, 서비스 요청(`current_pose`, `reference_path`)과 동일한 로컬 프레임(`~frame_id/local_frame`)로의 변환(또는 입력을 그 프레임으로 표준화)을 고려하세요. 변환이 어렵다면 최소한 프레임 체크 후 불일치 시 경고 및 보수적 제동 명령을 반환하도록 처리하세요.
- 임포트 의존성: `casadi`, `acados_template`, `l4casadi`, `torch`, `laser_line_extraction` 이 모듈 레벨에서 임포트됩니다. 일부 미설치 시 노드가 시작되지 않습니다. 선택 의존성(예: `laser_line_extraction`) 은 try/except 로 감싸고 미사용 시 구독을 비활성화하는 옵션을 고려하세요.
- 서비스 이름: 서비스는 절대 경로 `/psdf_mpc` 로 광고됩니다(전역 서비스). 런치/테스트에서 ns 를 부여하지 않습니다.
- 설정 오버라이드: `setup_optimizer()` 에서 `cfg.tf = cfg.horizon * self.params['dt']` 등 ROS 파라미터로 오버라이드합니다. `optimizer_config_file` 존재 여부가 확실치 않으면 빈 문자열을 사용하세요.
- 성능: acados 최초 생성은 느릴 수 있습니다. 노드 시작 시 워밍업(이미 수행) 또는 초기 지연에 대한 문서를 고려하세요.
- 로깅: 서비스 핸들러 로그가 INFO 레벨에서 다소 상세합니다. 안정화 후 일부를 DEBUG 로 낮추는 것을 고려하세요.

MPC 옵티마이저(파이썬)
- 궤적 형상: `AcadosSolution.get_input_trajectory()` 는 `(nu, N)` 형상을 반환합니다. `[0,0]`, `[1,0]` 접근은 맞습니다. 일관성 유지하세요.
- 솔버 상태: [fix] `AcadosSolution.stats()` 는 `self.solver.status` 를 참조하는데, `AcadosOcpSolver` 에 해당 속성이 항상 존재하지 않을 수 있습니다. `self.solver.solve()`의 반환 코드 보관 또는 `acados_status()`/`get_stats()` 사용으로 성공/실패 판정을 안정화하세요. 현재 코드는 예외 시 서비스가 실패로 응답하지만, 상태 판정 로직을 보강하는 것이 바람직합니다.
- JSON/코드젠 파일: 정리는 `acados_ocp.json`, `c_generated_code/` 를 커버합니다. 좋은 관행이며, 다중 노드 동시 실행 시 레이스가 없도록 유의하세요.
- 장애물 업데이트: `edge_cb` 가 `psdf_wrapper.update_edge_clusters` 로 PSDF 를 업데이트합니다 — 텐서 디바이스(`cpu`) 가 래퍼 생성과 일치하는지 확인(현재 CPU).

Launch/Config/Test 일관성
- 서비스 네임스페이스: 현재 런치/테스트는 전역 `/psdf_mpc` 서비스를 사용합니다. 이동 베이스와 함께 사용할 때 네임스페이스를 적용하려면 플래너 파라미터 `psdf_mpc_service`를 네임스페이스 포함 절대 경로로 맞추거나, 서비스는 전역으로 유지하세요(혼선 방지).
- 플래너 서비스 파라미터: [fix] `move_base/PSDFLocalPlanner` 파라미터에 `psdf_mpc_service: "psdf_mpc"` 를 추가하거나, 네임스페이스를 유지한다면 절대 경로 `/robot/psdf_mpc` 로 설정하세요.
- 프레임: `config/psdf_local_planner.yaml` 의 `frame_id/local_frame` 이 서비스/노드가 사용하는 프레임(옵티마이저는 설정에 따라 `odom` 또는 `base_link`)과 일치하는지 확인하세요.
  - 특히 테스트에서는 장애물 토픽(`/detected_edges`)의 `header.frame_id`가 `odom`인 반면, 노드 기본 `local_frame`은 `base_link`입니다. 런치에서 `frame_id/local_frame`을 명시적으로 `odom`으로 설정했는지 확인하고, 시각화/최적화 입력 프레임을 통일하세요.
- 파이썬 버전: shebang 이 `python3` 입니다. ROS Noetic 사용 또는 Python 3 환경을 보장하세요(kinetic/melodic 사용 시 조정 필요).

빠른 검증 체크리스트
- 빌드: C++11 활성화 및 플러그인 라이브러리 빌드/설치가 성공하고, `psdf_local_planner_plugin.xml` 이 share에 설치됩니다.
- 서비스: 최소 요청(경로 포함)으로 `rosservice call /psdf_mpc` 가 동작하며, 테스트 기준 ~1초 내 응답합니다.
- 플래너+서비스: move_base 가 `psdf_local_planner/PSDFLocalPlanner` 를 로드하고, 플래너가 서비스를 호출하여 플랜이 있을 때 `cmd_vel` 을 발행합니다.
- 테스트: 서비스 네임스페이스 정렬 후 `rostest psdf_ros test/test_psdf_node.launch` 가 통과합니다.

권장 최소 코드 변경
1) CMake: `add_compile_options(-std=c++11)` 주석 해제.
2) C++: 기본값을 `service_name_ = "/psdf_mpc";` 로 두고, 필요 시 `service_timeout_` 활용.
3) Python: 기본 footprint 콤마 수정 및 마커 프레임에 `self.params['local_frame']` 사용.
4) 테스트/launch: 전역 `/psdf_mpc` 사용, `robot` ns 제거.
