# fastapi_app.py

import uvicorn

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.templating import Jinja2Templates
from pwnbotcontrols.talker import Talker
from arm import EasyArm
import os

# from camera_single import Camera
from camera_multi import Camera

arm = EasyArm(12, 13)
app = FastAPI()
t = Talker()


# app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse('index.html', {"request": request})


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.get('/video_feed', response_class=HTMLResponse)
async def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return StreamingResponse(gen(Camera()),
                             media_type='multipart/x-mixed-replace; boundary=frame')


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            if data == "up":
                t.call("go_forward()")
            if data == "stop_up":
                t.call("stop()")

            if data == "down":
                t.call("go_backward()")
            if data == "stop_down":
                t.call("stop()")

            if data == "right":
                t.call("rotate_right()")
            if data == "stop_right":
                t.call("stop()")

            if data == "left":
                t.call("rotate_left()")
            if data == "stop_left":
                t.call("stop()")

            if data == "arm_up":
                arm.setHeight(arm.getHeight() + 5)

            if data == "arm_down":
                arm.setHeight(arm.getHeight() - 5)

            await websocket.send_text(f"Message text was: {data}")
    except WebSocketDisconnect:
        # stop movements
        t.call("stop()")
        print('websocket-disconnect')



if __name__ == "__main__":
    print('stop: ctrl+c')
    os.system("mpremote run ./pwnbotcontrols/motor_code.py")
    os.system("mpremote run ./pwnbotcontrols/motor_code.py")
    t.call('motor_initialize()')
    uvicorn.run(app, host="0.0.0.0", port=8000)