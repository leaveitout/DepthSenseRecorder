#include <iostream>

#include <DepthSense.hxx>

using namespace std;
using namespace DepthSense;

static void error(const char* message)
{
    std::cerr << message << std::endl;
}

// TODO: Refactor this so that it will work for multiple cameras
static ColorNode getFirstAvailableColorNode (Context context)
{
  // obtain the list of devices attached to the host
  vector<Device> devices = context.getDevices();

  for (vector<Device>::const_iterator iter = devices.begin(); iter != devices.end(); iter++)
  {
      Device device = *iter;

      // obtain the list of nodes of the current device
      vector<Node> nodes = device.getNodes();

      for (vector<Node>::const_iterator nodeIter = nodes.begin(); nodeIter != nodes.end(); nodeIter++)
      {
          Node node = *nodeIter;

          // if the node is a DepthSense::ColorNode, return it
          ColorNode colorNode = node.as<ColorNode>();
          if (colorNode.isSet())
              return colorNode;
      }
  }

  // return an unset color node
  return ColorNode();
}

static DepthNode getFirstAvailableDepthNode (Context context)
{
    // obtain the list of devices attached to the host
    vector<Device> devices = context.getDevices();

    for (vector<Device>::const_iterator iter = devices.begin(); iter != devices.end(); iter++)
    {
        Device device = *iter;

        // obtain the list of nodes of the current device
        vector<Node> nodes = device.getNodes();

        for (vector<Node>::const_iterator nodeIter = nodes.begin(); nodeIter != nodes.end(); nodeIter++)
        {
            Node node = *nodeIter;

            // if the node is a DepthSense::DepthNode, return it
            DepthNode depthNode = node.as<DepthNode>();
            if (depthNode.isSet())
                return depthNode;
        }
    }

    // return an unset depth node
    return DepthNode();
}

static void onNewColorSample (ColorNode obj, ColorNode::NewSampleReceivedData data)
{
    cout << "New color sample received (timeOfCapture=" << data.timeOfCapture << ")" << endl;
}


static void onNewDepthSample (DepthNode obj, DepthNode::NewSampleReceivedData data)
{
    cout << "New depth sample received (timeOfCapture=" << data.timeOfCapture << ")" << endl;
}

static ColorNode::Configuration configureColorNode(ColorNode node)
{
    ColorNode::Configuration config = node.getConfiguration();

    config.frameFormat = FRAME_FORMAT_VGA;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.framerate = 30;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;

    return config;
}

int main (int argc, char** argv)
{
    // create a connection to the DepthSense server at localhost
    Context context = Context::create();

    // get the first available color sensor
    ColorNode colorNode = getFirstAvailableColorNode(context);

    colorNode.setEnableCompressedData(true);

    // get the first available depth sensor
    DepthNode depthNode = getFirstAvailableDepthNode(context);

    // if no color node was found, fail
    if (! colorNode.isSet())
        error("no color node found");

    // if no depth node was found, fail
    if (! depthNode.isSet())
        error("no color node found");

    context.requestControl(colorNode);

    colorNode.setConfiguration(configureColorNode(colorNode));

    configureColorNode(colorNode);
    // enable the capture of the color map
    colorNode.setEnableColorMap(true);

    // connect a callback to the newSampleReceived event of the color node
    colorNode.newSampleReceivedEvent().connect(onNewColorSample);

    // enable the capture of the depth map
    depthNode.setEnableDepthMap(true);

    // connect a callback to the newSampleReceived event of the color node
    depthNode.newSampleReceivedEvent().connect(onNewDepthSample);

    // add the color node to the list of nodes that will be streamed
    context.registerNode(colorNode);

    // add the depth node to the list of nodes that will be streamed
    context.registerNode(depthNode);

    // start streaming
    context.startNodes();

    // start the DepthSense main event loop
    context.run();
}