plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("javax.vecmath:vecmath:1.5.2")
   api("com.google.guava:guava:18.0")

   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:ihmc-robot-description:0.21.3")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:scs2-definition:0.5.1-bullet-alpha-1")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {

}
