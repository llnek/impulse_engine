"use strict";
const gulp = require('gulp');
const uglify = require('gulp-terser');
//const notify = require('gulp-notify');
const concat = require('gulp-concat');
const rename = require('gulp-rename');
//const sourcemaps = require('gulp-sourcemaps');

const jsFiles = [
  "src/main/impulse.js",
  "src/main/shape.js",
  "src/main/body.js",
  "src/main/manifold.js",
  "src/main/collision.js",
  "src/main/scene.js"
];

var destDir = 'dist'; //or any folder inside your public asset folder
//To concat and Uglify All JS files in a particular folder
gulp.task("bundleJS", function(){
    return gulp.src(jsFiles)
        .pipe(concat("impulse_engine.js")) //this will concat all the files into concat.js
        .pipe(gulp.dest("dist")) //this will save concat.js in a temp directory defined above
        .pipe(uglify()) //this will uglify/minify uglify.js
        .pipe(rename("impulse_engine.min.js")) //this will rename concat.js to uglify.js
        .pipe(gulp.dest("dist")); //this will save uglify.js into destination Directory defined above
});

gulp.task("default", gulp.series("bundleJS"), function(){
  console.log("Gulp started");
});


