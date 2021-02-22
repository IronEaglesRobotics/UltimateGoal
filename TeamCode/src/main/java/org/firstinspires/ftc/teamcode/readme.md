# Iron Eagles Robotics 3708 Teamcode

[To-Do List](#to-do)

General Stuff
[How the code is structured](#how-the-code-is-structured)
[Adding your own code](#adding-your-own-code)
[Uploading code to the robot over USB-C](#uploading-code-to-the-robot-over-usb-c)
[Uploading code to the robot over WI-FI](#uploading-code-to-the-robot-over-wi-fi)

Tutorials
[Java quick start for beginners](#java-quick-start-for-beginners)
[Some other things that could be helpful to know](#some-other-things-that-could-be-helpful-to-know)

TL;DR
If you don't understand something in the code, read through it again and look for where it is used and fits into everything else, ask questions if you're stuck, and make the robot do cool stuff

## TO-DO
- [ ] fix turns
- [ ] fix strafing to goal
- [ ] move faster with slowing down at the end
- [ ] use color sensor for detecting white line
- [ ] add a `Programming a robot quick start for beginners` section to this
- [ ] make this readme better in general
- [ ] Longer term goals:
	- [ ] use PID
	- [ ] look into adding odometry or using the accelerometer on the IMU
	- [ ] look into a 360 camera or some way to always and accurately tell where you are on the field

## How the code is structured:  
- All of the code that we work on is underneath the teamcode folder, everything else is a magical black box that makes the app work
- In the [opmodes](opmodes) folder there are different opmodes, some for autonomous and some for driver controlled ones. The names should be pretty self-explanatory
- The opmodes use the classes that make up the robot from the [robot](robot) folder to direct the robot to make certain movements
- the [opencv](opencv) folders contains classes for detection of something that our code recognizes as something, and the pipelines contain methods that run every time a frame is received from the cameras
- the [CVHelpers](CVHelpers.java) and [MathHelpers](MathHelpers.java) contain helper methods for the rest of the code
- the [Constants](Constants.java) class contains a list of all the major constants that don't change
  
## Adding your own code:  
1. download android studio and clone (copy) this current git repository to somewhere local on your computer  
2. ask Senor Scott or Mr. Thurlow to give you your own branch if you don't already have one so we don't all try to change the same branch of code  
3. add your own code (try to add comments, make it readable, and make it something that you think will work / be an improvement over what was there before)  
4. press `Alt + S` and then `Ctrl + K` or click `VCS` and `Commit...` at the top
5. add your commit message (what you changed), then press `Ctrl + Alt + K` or the `Commit and Push` button to push your code
6. remember to **just commit to your own branch** unless you have permission otherwise, feel free to merge other branches into your own or suggest code that you think other people should include in their branches  
7. overall have fun and enjoy coding!

## Uploading code to the robot over USB-C:  
1. First connect to the robot control hub via usb-c
2. Open your code to upload in Android Studio
3. Press `Shift + F10` or click the green triangle to upload your code

## Uploading code to the robot over WI-FI:  
1. First connect to the FTC-prN network (password is password)  
2. Then run `adb connect 192.168.43.1:5555` in the terminal (you may have to specify the specific path for where adb is located)  
3. If for some reason it doesn't connect, connect via usb-c and run `adb tcpip 5555` in the terminal to restart the port in tcp mode and try again
4. Open your code to upload in Android Studio
5. Press `Shift + F10` or click the green triangle to upload your code

## Java quick start for beginners:
**Java**
Android is mainly written in Java, and because the FTC robot controller app is an android app it is coded in Java. We use Android Studio which is built for programming android apps in Java which when we are happy with the code it compiles it into machine code and then we upload it to the robot to run as the app. Below is my attempt at a Java tutorial for anyone looking to jump start learning Java.
**Comments**
Before we start going through Java, the first thing to look at is some basic syntax. Each line in Java is treated as code unless it is denoted as a comment. A comment is ignored by the compiler and is only there to help in readability purposes for you and other programmers.
```java
// this is a comment
/*
and
this
is
a
block
comment
*/
```
Each line of code should end with a `;` unless is is a `{`'`}`
**Variables and comments**
You can image variables as a box which you can assign a certain value, and you can then call that value later on and change it if needed. In Java there are 8 different primitive data types, and they are called this because they are as simple as you can get and don't have any special things added to them.
```java
// primitive data types
byte byteVar = 10;
short shortVar = 5000;
int intVar = 300000;
long longVar = 7000000000L;
float floatVar = 1.25f;
double doubleVar = 5.69392d;
boolean booleanVar = true;
char charVar = 'A';

// strings are special
String stringVar = "hello there";

// common list datatypes
int[] intArray = {1, 2, 3}
ArrayList<int> intArrayList= new ArrayList<>();
intArrayList.add(4);
intArrayList.add(5);
intArrayList.add(6);

// math with numbers
int a = 100;
int b = 7;
int c = a + b;
int d = a - b;
int e = a * b;
int f = a / b;
int g = a % b;

// concatenation with Strings
String s1 = "hello";
String s2 = "there";
String s3 = s1+" "+s2;

// converting datatypes
String string = "This is a string";
double number = 5.0;
int newNumber = (int) number;
```
Notice that the variables have to start with a certain type declaring what type of variable it is going to be, if it is an integer than it should be declared as an `int`, a decimal number as a `double`, etc. Each line in java ends with a `;` to let the compiler know that that line is finished. Lines that have `//` in them are treated as comments and don't run in the program, they are only there for readability purposes.

**If statements and switches**
If statements are a really easy way to do different things depending on a conditional, and take the syntax shown below:
```java
if (conditional) {
	// do something
} else if (other conditional) {
	// do a different thing;
} else {
	// resort to doing this;
}
```
You can also use something called a ternary operator which is just a shorthand way of doing an if statement:
```java
String message = (number % 2 == 0) ? "Even!" : "Odd!";
```
```java
String message = "";
if (number % 2 == 0) {
	message = "Even!";
} else {
	message = "Odd!";
}
```
If you want to do something different depending on the value of just one variable, a switch statement could come in handy
```java
switch(expression) {
  case x:
    // if expression is x, do this
    break;
  case y:
    // if expression is y, do this
    break;
  default:
    // if expression is neither x or y, do this
}
```
Just remember to put breaks in it, because otherwise once a case finishes, the code will automatically do the next case which normally is unwanted.

**For and while loops**
If you want to repeat something multiple times, you can use a for loop which in java looks like
```java
for (int i = 0; i < 10; i++) {
	System.out.println(i);
}
```
which will print out the numbers 0 through 9 to the console output

You can also use a while loop which can repeat until something evaluates to false
```java
while (conditional) {
	// keep doing something until the conditional is false
}
```

**Methods**
Methods in java  are a way to reuse code over an over again so you only have to type it out once
```java
public void sayHi() {
    System.out.println("Hi!");
}
```
in this case you can just type `sayHi()` in your program and it will run whatever code is inside of that method

methods can take parameters that you put inside the parentheses, which can be used inside of the function. Here is a function to add two numbers together:
```java
public int addXY(int x, int y) {
    return x + y;
}
```
Notice how it has an `int` instead of `void` because the function returns an integer value. In the first function it didn't return anything, so it have a void on it. The `public` keyword will be talked about in the next section.

**Classes**
Java is meant to be programmed using object oriented programming, otherwise known as OOP. All this basically means is that you can make a class like Dog and then you can make objects of that class, Bella and Max which are both dogs. They will both *inherit* the methods and values from the dog class, which will make the code easier to read and less tedious to write.
```java
class Dog() {
	String name;
	
	// This is the constructor
	public Dog(String name) {
		this.name = name;
	}
	
	// This is a method to bark
	public void Bark() {
		System.out.println("Bark bark bark!");
	}
}
```
```java
Dog bella = new Dog("Bella");
Dog max = new Dog("Max");
bella.bark();
max.bark();
```

**Putting it all together**
In most java programs it will have a `main` method, which will be called when the program starts.
```java
import java.util.ArrayList;

public class Program {
	public static void main(String args[]) {
		// primitive data types
		byte byteVar = 10;
		short shortVar = 5000;
		int intVar = 300000;
		long longVar = 7000000000L;
		float floatVar = 1.25f;
		double doubleVar = 5.69392d;
		boolean booleanVar = true;
		char charVar = 'A';
		
		// strings are special
		String stringVar = "hello there";
		
		// common list datatypes
		int[] intArray = {1, 2, 3}
		ArrayList<int> intArrayList= new ArrayList<>();
		intArrayList.add(4);
		intArrayList.add(5);
		intArrayList.add(6);

		// math with numbers
		int a = 100;
		int b = 7;
		int c = a + b;
		int d = a - b;
		int e = a * b;
		int f = a / b;
		int g = a % b;

		// concatenation with Strings
		String s1 = "hello";
		String s2 = "there";
		String s3 = s1+" "+s2;

		// converting datatypes
		String string = "This is a string";
		double number = 5.0;
		int newNumber = (int) number;
		
		// if, else if, else statement
		int time = 22;
		if (time < 10) {
			System.out.println("Good morning");
		} else if (time < 20) {
			System.out.println("Good day");
		} else {
			System.out.println("Good evening");
		}
		
		// ternary operator
		String result = (time < 18) ? "Good day" : "Good evening";
		System.out.println(result);
		
		// Switch statement in a method
		String event = getEvent("saturday");
		System.out.println(event+" will happen on saturday");
		
		// for loop
		for (int i = 0; i < 5; i++) {
			System.out.print(String.valueOf(i)+" ");
		}
		
		// while loop
		int j;
		while (j < 5) {
			System.out.print(String.valueOf(j));
			j++;
		}
		
		Dog bella = new Dog();
		System.out.println(bella.getName());
		bella.setName("bella");
		System.out.println(bella.getName());
		
		Dog max = new Dog("max");
		System.out.println(max.getName());
		System.out.println(max.bark());

		
	}
	public static String getEvent(String day) {
		switch(day) {
			case "saturday":
				return "robotics at savio";
				break;
			case "sunday":
				return "mass";
				break; 
			default:
				return "school";
		}
	}
}

public class Dog {
	// this is an attribute
	private String name;
	// these are the constructors
	public Dog() {
		this.name = "Doggo";
	}
	public Dog(String name) {
		this.name = name;
	}
	// this is a method to bark
	public void Bark() {
		System.out.println("Bark bark bark!");
	}
	// this is a method to get its name
	public String getName() {
		return name;
	}
	// this is a method to set its name to something else
	public void setName(String name) {
		this.name = name;
	}
}
```
Notice the main method has parameters which is an array called `args` which is any extra input that is called with the program. `static` methods are able to be called from the class, where as non static methods are for classes that have objects from them. It is public to let other programs be able to call the main method.

https://www.oracle.com/technetwork/java/codeconventions-150003.pdf

## Some other things that could be helpful to know:  
PID Control Systems
: Proportional Integral Derivative Controllers, otherwise known as PID Controllers, are commonly used in many applications that require changing something to another value as fast and accurately as possible. [Here](https://www.youtube.com/watch?v=JEpWlTl95Tw) is a good video on this, and is probably going to be the best way instead of me trying to explain it because I am still new to it. This is just a way to control the speed of the motors to reach the optimal point as fast as possible, currently we just have a Proportional (P\) controller, and are working towards a full PID system.

OpMode
: An OpMode contains code to control the robot that is run inside of the FTC app, and is what we work on. The two types are a normal OpMode for driver control where everything is inside of a big loop, and a LinearOpMode which goes through step by step for an autonomous program.

HardwareMap
: In order for the code to be able to access the motors on the robot, there is something called the HardwareMap which can be configured on the phone connected to the control hub. This configuration links a name to every motor, sensor, and camera so our code is able to find them.

IMU
: An IMU is included on the robot control hub which has lots of different sensors like a gyroscope that we can use to help control the robot.

Encoders
: For each motor you can plug in an encoder cable which can keep track of motor movements, and you can set the motor to run to different positions and get the number of ticks that it has rotated.