# 5970 Robot Code

It's the magic words to make the wheels go spin!

## Install Process

I set this up in kind of a silly way, so it's a little bit annoying to get working on a new machine. Here's how to do it anyway:

### Prereqs

* WPILib should be installed [(Tutorial and Download link Here)](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html#wpilib-installation-guide)
* Github Desktop should be installed [Github Desktop](https://desktop.github.com/)
    (Unless you want to wrangle git manually)  

Hopefully, you should have gotten to this file by using Clone Repository in github desktop.

Normally, this would be the end of it- You'd be able to write code from here. But unfortunately, the copy of the code you have here doesn't actually have all the files it needs. In order to get them, you need to create a temporary project using wpilib and steal them from there.

1. Hit Control-Shift-P, and begin typing "create a new project" until you can click on the dropdown option `WPILib: Create a new project`.
2. Enter in the details- File location doesn't matter, but make sure you select one of the java templates, enter `5970` as the team number, and check the desktop support tickbox.
3. Copy over the following files to their corrosponding locations:
    * gradlew
    * build.gradle
    * gradlew.bat
    * settings.gradle
    * gradle/wrapper/gradle-wrapper.jar
4. Reset Java by hitting Control-Shift-P and begin typing `Java: Clean Language Server Workspace` until you can select it from the list.
5. Run a test build by doing Control-Shift-P and typing `WPILib: Build Robot Code` until you can select it from the list.
6. It will pop open a terminal and it should give you a green loading bar made of `=` signs, and tell you
