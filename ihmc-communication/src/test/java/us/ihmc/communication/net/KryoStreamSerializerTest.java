package us.ihmc.communication.net;

import static us.ihmc.robotics.Assert.*;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class KryoStreamSerializerTest
{

	@Test
   public void test() throws IOException
   {
      KryoStreamSerializer kryoStreamSerializer = new KryoStreamSerializer(14);
      KryoStreamDeSerializer kryoStreamDeSerializer = new KryoStreamDeSerializer(14);
      kryoStreamSerializer.registerClass(A.class);
      kryoStreamSerializer.registerClass(B.class);
      kryoStreamSerializer.registerClass(C.class);
      
      kryoStreamDeSerializer.registerClass(A.class);
      kryoStreamDeSerializer.registerClass(B.class);
      kryoStreamDeSerializer.registerClass(C.class);

      A aIn = new A();
      B bIn = new B();
      C cIn = new C();

      aIn.data = 241;
      bIn.data = (long) 1e42;
      cIn.data = 8;

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();

      kryoStreamSerializer.write(outputStream, aIn);
      kryoStreamSerializer.write(outputStream, bIn);
      kryoStreamSerializer.write(outputStream, cIn);

      ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(outputStream.toByteArray());

      Object aOut = kryoStreamDeSerializer.read(byteArrayInputStream);
      Object bOut = kryoStreamDeSerializer.read(byteArrayInputStream);
      Object cOut = kryoStreamDeSerializer.read(byteArrayInputStream);

      assertTrue(aIn.getClass().equals(aOut.getClass()));
      assertTrue(bIn.getClass().equals(bOut.getClass()));
      assertTrue(cIn.getClass().equals(cOut.getClass()));

      assertEquals(aIn.data, ((A) aOut).data);
      assertEquals(bIn.data, ((B) bOut).data);
      assertEquals(cIn.data, ((C) cOut).data);

   }

   private static class A
   {
      public int data;
   }

   private static class B
   {
      public long data;
   }

   private static class C
   {
      public byte data;
   }

}
