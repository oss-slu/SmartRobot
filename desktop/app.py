from flask import Flask, render_template, jsonify

app = Flask(__name__)

# Robot status: "ON" or "OFF"
robot_status = "OFF"

@app.route("/")
def index():
    return render_template("index.html")

# My Robot page
@app.route("/my-robot")
def my_robot():
    return render_template("my_robot.html")

@app.route("/status")
def status():
    return jsonify({"status": robot_status})

# Let WebSocket commands update status
@app.route("/set_status/<new_status>")
def set_status(new_status):
    global robot_status
    if new_status.upper() in ["ON", "OFF"]:
        robot_status = new_status.upper()
    return jsonify({"status": robot_status})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)
