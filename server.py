from flask import Flask, request, render_template, jsonify

app = Flask(__name__)

message = "Waiting for updates..."  # Default message

@app.route("/")
def home():
    return render_template("index.html", message=message)

@app.route("/update", methods=["POST", "GET"])
def update_message():
    global message
    
    if request.method == "POST":
        data = request.get_json()
        if not data or "default" not in data:
            return jsonify({"error": "Invalid request"}), 400
        message = data["default"]
        return jsonify({"status": "success", "updated_message": message}), 200

    # Handle GET request (for webpage auto-refresh)
    return jsonify({"updated_message": message})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
