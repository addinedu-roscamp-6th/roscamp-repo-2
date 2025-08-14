-- pinkyDB.sql (SQLite용)
-- 노인 요양원 자동 배식 시스템

-- 1. 외래 키 활성화 (SQLite 기본 비활성화)
PRAGMA foreign_keys = ON;

-- 2. 테이블 생성

-- 환자 테이블
CREATE TABLE patients (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    room_number TEXT NOT NULL,
    dietary_restriction TEXT,
    doctor_notes TEXT,
    created_at TEXT DEFAULT (datetime('now'))
);

-- 메뉴 항목 테이블
CREATE TABLE menu_items (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    category TEXT CHECK(category IN ('breakfast', 'lunch', 'dinner')) NOT NULL,
    calories INTEGER,
    nutrition_info TEXT,
    is_vegetarian BOOLEAN DEFAULT 0,
    is_low_salt BOOLEAN DEFAULT 0,
    is_diabetic_friendly BOOLEAN DEFAULT 0
);

-- 식사 계획 테이블
CREATE TABLE meal_plans (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    patient_id INTEGER NOT NULL,
    menu_item_id INTEGER NOT NULL,
    meal_date TEXT NOT NULL,
    meal_type TEXT CHECK(meal_type IN ('breakfast', 'lunch', 'dinner')) NOT NULL,
    status TEXT DEFAULT 'planned',
    created_at TEXT DEFAULT (datetime('now')),
    FOREIGN KEY (patient_id) REFERENCES patients(id) ON DELETE CASCADE,
    FOREIGN KEY (menu_item_id) REFERENCES menu_items(id) ON DELETE CASCADE,
    UNIQUE (patient_id, meal_date, meal_type)
);

-- 배달 로봇 테이블
CREATE TABLE delivery_robots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id TEXT UNIQUE NOT NULL,
    status TEXT CHECK(status IN ('idle', 'delivering', 'charging', 'offline')) DEFAULT 'idle',
    current_location TEXT,
    battery_level INTEGER,
    last_updated TEXT DEFAULT (datetime('now'))
);

-- 배달 작업 테이블
CREATE TABLE delivery_tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    meal_plan_id INTEGER NOT NULL,
    robot_id INTEGER,
    start_time TEXT,
    end_time TEXT,
    status TEXT CHECK(status IN ('pending', 'in_progress', 'completed', 'failed')) DEFAULT 'pending',
    delivery_notes TEXT,
    created_at TEXT DEFAULT (datetime('now')),
    FOREIGN KEY (meal_plan_id) REFERENCES meal_plans(id) ON DELETE CASCADE,
    FOREIGN KEY (robot_id) REFERENCES delivery_robots(id) ON DELETE SET NULL
);

-- 시스템 로그 테이블
CREATE TABLE system_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    log_time TEXT DEFAULT (datetime('now')),
    level TEXT CHECK(level IN ('info', 'warning', 'error')),
    message TEXT NOT NULL,
    source TEXT
);

-- 3. 테스트 데이터 삽입

INSERT INTO patients (name, room_number, dietary_restriction) VALUES 
('김할머니', '101', '저염, 해산물 금지'),
('이할아버지', '102', '당뇨병 식단');

INSERT INTO menu_items (name, category, calories, is_diabetic_friendly) VALUES 
('현미 죽', 'breakfast', 180, 1),
('고등어 구이', 'lunch', 320, 0),
('두부 샐러드', 'dinner', 190, 1);

INSERT INTO delivery_robots (robot_id, status, battery_level, current_location) VALUES 
('RP001', 'idle', 90, '주방');

INSERT INTO meal_plans (patient_id, menu_item_id, meal_date, meal_type) VALUES 
(1, 2, '2025-04-05', 'lunch'),
(2, 1, '2025-04-05', 'breakfast');

INSERT INTO delivery_tasks (meal_plan_id, robot_id, status) VALUES 
(1, 1, 'pending'),
(2, 1, 'pending');

-- 완료 메시지
SELECT '✅ 노인 요양원 자동 배식 시스템 데이터베이스 생성 완료!' AS Status;
