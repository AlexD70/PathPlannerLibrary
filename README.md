[Vlad Chira's PathPlanner](https://github.com/VladChira/PathPlannerJava) with some little changes.

* removed module-info.java (that thing doesn't play well with gradle - or it is past my abilities to make it work)
* added gradle scripts
* removed javafx app as this is meant to be a library for use in FTC and having a runtime dependency on javafx doesn't sound great
* pom.xml removed since it may confuse IDEs when found alongside gradle scripts