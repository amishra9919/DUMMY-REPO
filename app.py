from flask import Flask, request, jsonify
from PIL import Image
import io
import base64

app = Flask(__name__)

@app.route('/grayscale', methods=['POST'])
def grayscale():
    # Get the image from the request
    file = request.files['image']
    image = Image.open(file)

    # Convert the image to grayscale
    grayscale_image = image.convert('L')

    # Save image to bytes buffer
    buf = io.BytesIO()
    grayscale_image.save(buf, format='PNG')
    byte_im = buf.getvalue()

    # Convert to base64 for JSON response
    img_str = base64.b64encode(byte_im).decode('utf-8')
    return jsonify({'image': img_str})

if __name__ == '__main__':
    app.run(debug=True)
