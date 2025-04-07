import time
import serial
import requests
import numpy as np
import cv2
import gspread
import os
from oauth2client.service_account import ServiceAccountCredentials
from pydrive2.auth import GoogleAuth
from pydrive2.drive import GoogleDrive
from requests.auth import HTTPBasicAuth
import atexit
# -------------------- Google Sheets API Setup --------------------
def auth_google_sheets(json_keyfile):
    """Authenticate with Google Sheets using a service account."""
    scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
    creds = ServiceAccountCredentials.from_json_keyfile_name(json_keyfile, scope)
    client = gspread.authorize(creds)
    return client
# JSON Key File Path (modify with actual path)
JSON_KEYFILE = r"/home/kdt/Downloads/aitest-450807-860ecea75554.json"
client = auth_google_sheets(JSON_KEYFILE)
# Google Sheets Connection
SPREADSHEET_NAME = "SMART_VISION_RK_DB"
sheet = client.open(SPREADSHEET_NAME).worksheet("DB")
def get_next_id(sheet):
    """Retrieve the last ID from Google Sheets and determine the next ID."""
    col_a_values = sheet.col_values(1)  # A열 값 가져오기
    if len(col_a_values) > 2:  # 데이터가 있으면 마지막 행의 ID를 가져옴 (헤더 제외)
        last_id = int(col_a_values[-1])  # 마지막 ID 가져오기
        return last_id + 1  # 다음 ID 반환
    else:  # 시트가 비어 있으면 1부터 시작
        return 1
def write_to_sheet(sheet, data):
    """Ensure data is written from A3 downward, keeping ID order correct."""
    col_a_values = sheet.col_values(1)  # A열 값 가져오기
    next_row = max(3, len(col_a_values) + 1)  # A3부터 시작 (최소 3번째 줄)
    sheet.update(f"A{next_row}", [data])  # A열부터 순차적으로 저장
# -------------------- Google Drive API Setup --------------------
def auth_google_drive():
    """Authenticate with Google Drive using a service account."""
    scope = ["https://www.googleapis.com/auth/drive"]
    creds = ServiceAccountCredentials.from_json_keyfile_name(JSON_KEYFILE, scope)
    gauth = GoogleAuth()
    gauth.credentials = creds
    return GoogleDrive(gauth)
drive = auth_google_drive()
def upload_to_drive(file_path, file_name):
    """Upload defective image to Google Drive and return the shared link."""
    file = drive.CreateFile({
        'title': file_name,
        'parents': [{"id": "13_WzO45EGgfYaLvz4_v3mB654miWhEHc"}]  # Modify parent folder ID if needed
    })
    file.SetContentFile(file_path)
    file.Upload()
    file.InsertPermission({'type': 'anyone', 'value': 'anyone', 'role': 'reader'})
    return file['alternateLink']
# -------------------- Serial Port Setup --------------------
ser = serial.Serial("/dev/ttyACM0", 9600)
# YOLO API Information
VISION_API_URL = "https://suite-endpoint-api-apne2.superb-ai.com/endpoints/143dc688-df5c-4578-abbc-c3f08bb7460f/inference"
TEAM = "kdt2025_1-31"
ACCESS_KEY = "XZ2NoPrSKpa8th9fTbXBV8fYelJjtjpP1RBlqcVK"
# -------------------- Image Processing --------------------
def get_img():
    """Capture image from USB camera."""
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Camera Error")
        exit(-1)
    ret, img = cam.read()
    cam.release()
    return img
def crop_img(img, size_dict):
    """Crop image based on given coordinates and size."""
    x, y, w, h = size_dict["x"], size_dict["y"], size_dict["width"], size_dict["height"]
    return img[y:y+h, x:x+w]
def save_img(img, file_path):
    """Save image to specified file path and return the path."""
    # Create folder if it doesn't exist
    folder = os.path.dirname(file_path)
    if folder and not os.path.exists(folder):
        os.makedirs(folder)
    cv2.imwrite(file_path, img)
    return file_path
def inference_request(img):
    """Send image to YOLO API and return the result."""
    success, img_encoded = cv2.imencode(".jpg", img)
    if not success:
        print("Image encoding failed.")
        return None
    data = img_encoded.tobytes()
    try:
        response = requests.post(
            url=VISION_API_URL,
            auth=HTTPBasicAuth(TEAM, ACCESS_KEY),
            headers={"Content-Type": "image/jpeg"},
            data=data
        )
        return response.json() if response.status_code == 200 else None
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")
        return None
def draw_boxes_on_image(img, inference_result):
    """Draw bounding boxes and labels on the image."""
    if inference_result is None or "objects" not in inference_result:
        return img
    for obj in inference_result["objects"]:
        cls, score = obj["class"], obj["score"]
        x1, y1, x2, y2 = obj["box"]
        start_point, end_point = (x1, y1), (x2, y2)
        color_dict = {
            'RASPBERRY PICO': (255, 0, 0),
            'BOOTSEL': (0, 255, 0),
            'USB': (0, 0, 255),
            'CHIPSET': (255, 0, 255),
            'HOLE': (255, 255, 0),
            'OSCILLATOR': (0, 165, 255)
        }
        color = color_dict.get(cls, (0, 255, 255))
        cv2.rectangle(img, start_point, end_point, color, 2)
        text = f"{cls}: {score:.2f}"
        cv2.putText(img, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return img
# -------------------- ID and Batch Timing Setup --------------------
local_id = 1         # Local index for each processed item (resets when code is re-run)
batch_count = 0      # Counter for batch (10 items per batch)
batch_start_time = time.time()  # Start time for current batch
# -------------------- Main Loop --------------------
atexit.register(cv2.destroyAllWindows)
while True:
    data1 = ser.read()
    print("Serial data received:", data1)
    if data1 == b"0":
        img = get_img()
        crop_info = {"x": 200, "y": 100, "width": 300, "height": 300}
        img = crop_img(img, crop_info)
        # Perform YOLO detection
        result = inference_request(img)
        img = draw_boxes_on_image(img, result)
        # If no detection result, skip this iteration
        if result is None or "objects" not in result:
            print("No objects detected!")
            continue
        # Define expected counts and confidence thresholds
        expected_counts = {
            'RASPBERRY PICO': 1,
            'BOOTSEL': 1,
            'CHIPSET': 1,
            'USB': 1,
            'OSCILLATOR': 1,
            'HOLE': 4
        }
        min_conf_thresholds = {
            'RASPBERRY PICO': 0.98,
            'BOOTSEL': 0.9,
            'CHIPSET': 0.9,
            'USB': 0.9,
            'OSCILLATOR': 0.87,
            'HOLE': 0.86
        }
        # Check actual counts and confidence thresholds
        actual_counts = {}
        is_normal = True
        error_logs = []
        for obj in result["objects"]:
            cls, score = obj["class"], obj["score"]
            actual_counts[cls] = actual_counts.get(cls, 0) + 1
            if score < min_conf_thresholds.get(cls, 0.5):
                is_normal = False
                error_logs.append(f"Low confidence in {cls}: {score:.2f}")
        for cls, expected in expected_counts.items():
            count = actual_counts.get(cls, 0)
            if count != expected:
                is_normal = False
                error_logs.append(f"Count mismatch for {cls}: expected {expected}, got {count}")
        status_text = "SUCCESS" if is_normal else "DEFECTIVE"
        # Draw status text on image for display
        text_color = (0, 255, 0) if status_text == "SUCCESS" else (0, 0, 255)
        cv2.putText(img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2, cv2.LINE_AA)
        # Compute detected count and defective count (total expected objects = 9)
        detected_count = len(result["objects"])
        total_expected = 9
        defective_count = 0 if is_normal else total_expected - detected_count
        # Prepare detected objects string
        detected_list = [f"{obj['class']} ({obj['score']:.2f})" for obj in result["objects"]]
        detected_str = "; ".join(detected_list)
        # Use local_id as the ID and increment it for each detection
        ID = local_id
        local_id += 1
        batch_count += 1
        # For defective cases, save image and upload to Google Drive
        drive_url = "N/A"
        if status_text == "DEFECTIVE":
            file_name = f"defective_{ID}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            defect_img_path = os.path.join("defective_images", file_name)
            save_img(img, defect_img_path)
            drive_url = upload_to_drive(defect_img_path, file_name)
        # Write results to Google Sheets
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        # Sheet columns: ID, Timestamp, Detected Count, Detected Objects, Defective Count, Status, Drive URL
        write_to_sheet(sheet, [ID, timestamp, detected_count, detected_str, defective_count, status_text, drive_url])
        print(f"Data saved to Google Sheets: ID={ID}, Timestamp={timestamp}, Detected={detected_str}, "
              f"Defective Count={defective_count}, Status={status_text}, Image URL={drive_url}")
        # Display image
        cv2.imshow("YOLO Detection Result", img)
        cv2.waitKey(1)
        # Send serial response
        ser.write(b"1")
        # Batch timing: every 10 items, print elapsed time and reset batch counter
        # if batch_count == 10:
        #     elapsed_time = time.time() - batch_start_time
        #     print(f"Processed 10 items in {elapsed_time:.2f} seconds.")
        #     batch_count = 0
        #     batch_start_time = time.time()
