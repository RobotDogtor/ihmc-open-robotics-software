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
   api("us.ihmc:euclid:0.17.2")
   api("org.ddogleg:ddogleg:0.18")

   api("us.ihmc:ihmc-yovariables:0.9.11")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
   api("us.ihmc:simulation-construction-set:0.21.16")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api("us.ihmc:ihmc-robotics-toolkit-test:source")
}
