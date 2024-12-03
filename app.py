#.\env\scripts\activate

from flask import Flask, render_template

app = Flask(__name__)

@app.route("/")           ##for different different pages
def hello_world():
    return 'Hello World';
    # pass
    # return render_template('index.html')
    # return "Hello World!"

# @app.route("/product")
# def products():          
#     return "This is products page!"  ##if default function i.e '@app.route("/")' is not returning anything then it will make this return as default

if __name__ == '__main__':
    app.run(debug=True)


