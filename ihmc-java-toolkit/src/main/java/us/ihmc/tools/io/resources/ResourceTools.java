package us.ihmc.tools.io.resources;

import org.apache.commons.io.IOUtils;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ConfigurationBuilder;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.InputStream;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Set;
import java.util.TreeSet;

/**
 * The system class loader, 99% of the time, is the same as your class's loader.
 * 
 * From Java documentation:
 * 
 * If a security manager is present, and the invoker's class loader is not null and the
 * invoker's class loader is not the same as or an ancestor of the system class loader,
 * then this method invokes the security manager's checkPermission method with a
 * RuntimePermission("getClassLoader") permission to verify access to the system class
 * loader. If not, a SecurityException will be thrown.
 */
public class ResourceTools
{
   /**
    * Path relative to source folders of clazz.
    */
   public static InputStream openStreamAbsolute(Class<?> clazz, Path path)
   {
      return clazz.getClassLoader().getResourceAsStream(path.toString());
   }
   
   /**
    * Path relative to clazz.
    */
   public static InputStream openStreamRelative(Class<?> clazz, Path path)
   {
      return clazz.getResourceAsStream(path.toString());
   }
   
   /**
    * Path relative to all source folders.
    */
   public static InputStream openStreamSystem(Path path)
   {
      return ClassLoader.getSystemResourceAsStream(path.toString());
   }
   
   /**
    * Path relative to source folders of clazz.
    */
   public static URL getResourceAbsolute(Class<?> clazz, Path path)
   {
      return clazz.getClassLoader().getResource(path.toString());
   }
   
   /**
    * Path relative to clazz.
    */
   public static URL getResourceRelative(Class<?> clazz, Path path)
   {
      return clazz.getResource(path.toString());
   }
   
   /**
    * Path relative to all source folders.
    */
   public static URL getResourceSystem(Path path)
   {
      return ClassLoader.getSystemResource(path.toString());
   }

   public static String readResourceToString(InputStream inputStream)
   {
      return ExceptionTools.handle(() -> IOUtils.toString(inputStream, StandardCharsets.UTF_8), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

   }
   public static String readResourceToString(URL url)
   {
      return ExceptionTools.handle(() -> IOUtils.toString(url, StandardCharsets.UTF_8), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static String readResourceToString(String path)
   {
      return readResourceToString(getResourceSystem(Paths.get(path)));
   }

   public static String sanitizeResourcePath(String resourcePath)
   {
      Path resourcePathAsPathRaw = Paths.get(resourcePath);
      Path firstPart = resourcePathAsPathRaw.getName(0);
      Path normalizedAbsoluteResourceFolderPath = firstPart.resolve(firstPart.relativize(resourcePathAsPathRaw).normalize());
      return normalizedAbsoluteResourceFolderPath.toString();
   }

   public static Set<String> listResources()
   {
      return listResources("", ".*");
   }

   /**
    * Using the Reflections library seems to be the only way to do this:
    * https://github.com/ronmamo/reflections
    */
   public static Set<String> listResources(String packagePathWithDots, String filterRegex)
   {
      Reflections reflections = new Reflections(new ConfigurationBuilder().forPackage(packagePathWithDots).setScanners(Scanners.Resources));
      String withSlashes = packagePathWithDots.replaceAll("\\.", "/");
      if (!withSlashes.isEmpty())
         withSlashes += "/";
      TreeSet<String> resources = new TreeSet<>();
      for (String resource : reflections.getResources(filterRegex))
      {
         if (resource.startsWith(withSlashes))
         {
            String subsequentPath = resource.replaceFirst(withSlashes, "");
            resources.add(subsequentPath);
         }
      }
      return resources;
   }

   public static boolean exists(Class<?> getClassForLoading, String name)
   {
      return getClassForLoading.getResource(name) != null;
   }

   public static boolean exists(String name)
   {
      return ClassLoader.getSystemResource(name) != null;
   }
}
