from flask import Blueprint, render_template, redirect, url_for

home_bp=Blueprint('home_bp',__name__)#create route for home
@home_bp.route('/')
def index():
    return redirect(url_for('pathfinding.select_points'))  # Redirect to point selection