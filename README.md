# Матрица компетенций разработчика робототехники

---

## 1 · Hard skills — профессиональные компетенции

### 1.1 Simulation & Digital Twin (NVIDIA Isaac Sim / Omniverse)
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.1.a PhysX & Scene Setup** | Импорт готовых USD/URDF, базовые коллайдеры, изменение массы. | Настраивает кастомные PhysX-материалы, сцены из нескольких роботов, автоматизирует через Python API. | Проектирует HIL‑тестовые полигоны, оптимизирует FPS >60, пишeт кастомные плаг‑сенсоры (RTX‑LiDAR, RTX‑Camera). | Путает USD c URDF, говорит «работал в Gazebo», нет профиля FPS/GPU. |
| **1.1.b Synthetic Data Gen** | Экспорт отдельных RGB/Depth кадров вручную. | Скрипты батч‑рендера, доменно‑рандомизирует освещение/текстуры. | Строит полный data‑pipeline, использует Replicator + NGC, генерирует >1 M img, трекает метрики датасета. | «Никогда не нужно было синтетическое», путает train/test split. |
| **1.1.c Omniverse Extensions** | Читает чужие sample‑ext. | Меняет UI‑панели, добавляет простую кнопку. | Пишет production‑ready extension, внедряет в CI, покрывает тестами. | «Не знаю, как собрать .kit‑extension», mixed QT + USD confusion. |

### 1.2 RL & Control (Isaac Lab + Orbit)
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.2.a Task Design** | Использует готовый Cartpole/Quadruped env. | Создаёт custom task + reward, tune‑ит hyperparams. | Оптимизирует vectorized env 1024×, внедряет curriculum & asymmetric obs. | «PPO = policy gradient?», копирует конфиг без понимания. |
| **1.2.b Training Pipeline** | Запускает rsl‑rl/rl_games на одном GPU. | Дебажит NaN, переносит на DGX, логирует TensorBoard. | Настраивает multi‑node DDP, mixed precision, online eval, автоматический sweep Optuna. | Считает, что replay‑buffer «не нужен для PPO», не знает, что такое seed. |
| **1.2.c Sim‑to‑Real Transfer** | Не сталкивался. | Добавляет domain randomization, P‑gain tuning по итогам real‑runs. | Использует RMA/UPOSI, отладил policy на реальном роботе (>1 ч устойчивой ходьбы). | Говорит «после симуляции всё сразу работает». |

### 1.3 Navigation & SLAM (ROS 2 Nav2 / Cartographer)
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.3.a Costmaps & BT‑Navigator** | Запускает turtlebot_world launch. | Тюнинг inflation/obstacles, пишет простой BT‑node. | Создаёт Hybrid A* planner для legged robot, оптимизирует recovery‑behaviors. | Думает, что Nav2 = «move_base», не знает BT‑Editor. |
| **1.3.b Localization** | GNSS + AMCL по туториалу. | Конфигурирует Cartographer 2D/3D, NDT‑matching. | Fuse‑ит IMU+Odometry+LiDAR (UKF), live‑relocalization. | Путает map / odom frames, «TF это про moveit». |
| **1.3.c Behavior Testing** | – | Пишет bag‑play сценарии, анализирует RQT‑graph. | Проводит Monte‑Carlo (>1000 runs) тесты, метрики success-rate в CI. | Нет систематики: «просто катаю руками». |

### 1.4 3‑D LiDAR & Perception
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.4.a Point‑Cloud Processing** | Фильтр VoxelGrid, passthrough. | Ground‑removal, clustering (Euclidean, DBSCAN). | Реализует ICP/NDT SLAM, CUDA‑акселерация, пишет ROS 2 nodelet. | Думает, что PointCloud2 = набор изображений. |
| **1.4.b Sensor Integration** | Транслирует пакеты VelodyneDriver. | Рефакторит time‑sync IMU + LiDAR, компенсирует motion‑distortion. | Разбирает Ouster UDP, low‑level diagnostics, калибровка extrinsics. | Не понимает разницы dual‑return/strongest, бросает пакеты. |
| **1.4.c Dataset & Metrics** | – | Записывает rosbag, использует pcl::PCLPointCloud2. | Вводит own metric (plane‑fit RMSE), автоматическая оценка в CI. | Нет валидации, «визуально хорошо». |

### 1.5 DevOps for Robotics
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.5.a Build & CI** | `colcon build`, линтер. | GitHub Actions, молекулярные тесты, artifact‑upload *.deb. | Bazel cross‑compile Jetson × x86, кеш BazelRemote, self‑hosted Runners. | «ROS 2 не надо собирать с –symlink‑install», вручную копирует. |
| **1.5.b Containers** | Dockerfile из примера. | Multi‑stage build, ROS_DOMAIN_ID env, compose для симуляции кластера. | Push в NGC, k8s Helm для Isaac Sim farm, GPU monitoring Prometheus. | Билдит ob‑образ >10 GB, игнорирует nvidia‑container‑runtime. |
| **1.5.c OTA & Fleet‑Ops** | – | Ansible update *.deb по SSH. | A/B slots, secure‑boot, Delta OTA, rollout strategy 50‑>1000. | Обновляет «вручную по ssh root@». |

### 1.6 System Engineering
| Под‑компетенция | Junior | Middle | Senior | Red Flags |
|-----------------|--------|--------|--------|-----------|
| **1.6.a Requirements** | Читает Confluence page. | Пишет SysML use‑cases, traceability matrix. | Ведёт DOORS‑NG, coverage >90 %, auto‑sync с tests. | «ТЗ — потом напишем». |
| **1.6.b Safety & RAMS** | – | Заполняет FMEA table, MTBF калькулятор. | Функц. безопасность ISO 13849 Cat 3, делает FTA pour‑over‑coffee. | Не знает diff between FMEA/FTA, «safety = e‑stop». |
| **1.6.c Certification** | – | Знает CE low‑voltage/EMC. | Планирует IEC 61508 SIL2, ведёт evidences. | «Никогда не сертифицировали». |

---

## 2 · Soft skills — личностно‑командные компетенции

| Компетенция | Junior | Middle | Senior | Red Flags |
|-------------|--------|--------|--------|-----------|
| **2.1 Системное мышление** | Видит связь «сенсор → алгоритм». | Рассматривает trade‑offs, эскалирует риски. | Строит multi‑доменные модели, управляет сложностью. | «Это не моё, пусть hardware решает». |
| **2.2 Экспериментальный подход** | Формулирует гипотезу A/B. | Ведёт табличку метрик, реплейсирует баг‑лоны. | Настраивает full pex‑pipeline, statistical power. | «Собираем данные, потом посмотрим». |
| **2.3 Коммуникация** | Пишет понятные PR‑description. | Делает RFC, схемы Miro/PlantUML. | Менторит, ведёт внутренние курсы, cross‑team alignment. | «Код говорит сам за себя», игнорирует review. |
| **2.4 Адаптивность** | Освоил новый SDK за неделю. | Пилотирует PoC, делится best‑practices. | Ведёт R&D‑roadmap, разворачивает новый стек целиком. | Паникует при смене приоритетов. |
| **2.5 Лидерство в зоне ответственности** | Берёт таск, доводит до Done. | Организует mini‑команду, драйвит demo. | Создаёт процессы, ретроспективы, улучшает культуру. | «Мне сказали, я сделал». |
| **2.6 Культура обратной связи** | Придерживается code‑style. | Дает конструктивный‑peer review. | Снимает блокеры, фасилитирует командное обсуждение. | Токсичность, blaming. |

---

## 3 · Опциональные компетенции (плюс к основным)

### 3.1 RTOS & Embedded (Zephyr / FreeRTOS)
| Junior | Middle | Senior | Red Flags |
|--------|--------|--------|-----------|
| Пишет LED‑blink, Makefile. | Драйвер CAN, RT‑задержки <1 ms, unit‑тесты Unity. | Пишет HAL для EtherCAT, safety watchdog, сертифицируемый код MISRA‑C. | Считает RTOS «лишней», прерывания «это сложно». |

### 3.2 Legged Locomotion (Kinematics / Control)
| Junior | Middle | Senior | Red Flags |
|--------|--------|--------|-----------|
| Z‑сплайны траектории, PID для hip. | Inverse‑dyn. IK (Whole‑Body Control), gait‑scheduler. | MPC / LQR torso‑stabilization, online terrain adaptation. | Говорит «четырёхногий = колёсный + lift foot». |

---

## 4 · Как использовать документ
1. **Создайте карточку вакансии**: выберите нужные под‑компетенции и минимальный уровень (Middle/Senior).  
2. **Скрининг резюме**: ищите подтверждения в портфолио, GitHub, видео‑демо.  
3. **Тех‑интервью**: берите примеры вопросов из таблиц Middle → Senior требуемого уровня.  
4. **Red Flags**: если обнаружен ≥1 красный флаг в ключевой компетенции, кандидат получает —1 уровень.

Документ поддерживается R&D‑тим‑лидами G1; предложения по улучшению присылайте в pull‑request к репозиторию `/docs/competency‑matrix`.  
© UNITREE G1, 2025

