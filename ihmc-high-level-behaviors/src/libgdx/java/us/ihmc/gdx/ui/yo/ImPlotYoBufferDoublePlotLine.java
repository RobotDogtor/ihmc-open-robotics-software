package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.simulation.scs2.GDXYoManager;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.text.DecimalFormat;
import java.util.function.Consumer;

public class ImPlotYoBufferDoublePlotLine extends ImPlotYoBufferPlotLineBasics
{
   private static final DecimalFormat decimal5DPrintFormatter = new DecimalFormat("0.00000");
   private final YoDouble yoDouble;
   private LinkedYoVariable<YoDouble> linkedYoDoubleVariable;
   private Integer[] xValues = ImPlotTools.createIndex(1);
   private Double[] plotData = ImPlotTools.newNaNFilledDoubleBuffer(1);

   public ImPlotYoBufferDoublePlotLine(YoDouble yoDouble, Consumer<YoVariable> removeSelf)
   {
      super(yoDouble, "NaN", removeSelf);
      this.yoDouble = yoDouble;
   }

   @Override
   public void setupLinkedVariable(GDXYoManager yoManager)
   {
      if (linkedYoDoubleVariable == null)
      {
         linkedYoDoubleVariable = (LinkedYoVariable<YoDouble>) yoManager.newLinkedYoVariable(yoDouble);
         linkedYoDoubleVariable.addUser(this);
      }
   }

   @Override
   public void update()
   {
      if (linkedYoDoubleVariable != null)
      {
         linkedYoDoubleVariable.pull();

         if (linkedYoDoubleVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample<double[]> bufferSample = linkedYoDoubleVariable.pollRequestedBufferSample();
            double[] buffer = bufferSample.getSample();
            int sampleLength = bufferSample.getSampleLength();
            if (plotData.length != sampleLength)
            {
               xValues = ImPlotTools.createIndex(sampleLength);
               plotData = ImPlotTools.newNaNFilledDoubleBuffer(sampleLength);
            }
            for (int i = 0; i < bufferSample.getBufferProperties().getActiveBufferLength(); i++)
            {
               plotData[i] = buffer[i];
            }
         }

         linkedYoDoubleVariable.requestEntireBuffer();
      }
   }

   @Override
   protected void plot(String labelID)
   {
      int offset = 0; // This is believed to be the index in the array we are passing in which implot will start reading
      ImPlot.plotLine(labelID, xValues, plotData, offset);
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return decimal5DPrintFormatter.format(plotData[bufferIndex]);
   }
}
