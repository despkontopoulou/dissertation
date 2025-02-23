from flask import Blueprint, render_template

home_bp=Blueprint('home_bp',__name__)#create route for home
@home_bp.route('/')
def index():
    return render_template("home/index.html")