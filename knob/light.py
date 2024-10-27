import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, light
from esphome.const import CONF_ID, CONF_OUTPUT_ID, CONF_OUTPUT, CONF_BLUE, CONF_GREEN, CONF_RED, CONF_BRIGHTNESS


AUTO_LOAD = [ "sensor", "light" ]
DEPENDENCIES = ["i2c"]

CONF_I2C_ADDR = 0x0F
CONF_ENCODER = "rotary_knob"
CONF_LIGHT = "knob_led"

myknob_ns = cg.esphome_ns.namespace("knob_ns")
MyKnobComponent = myknob_ns.class_('MyKnobComponent', light.LightOutput, i2c.I2CDevice, cg.PollingComponent)

CONFIG_SCHEMA = (
  light.RGB_LIGHT_SCHEMA.extend(
    {
      cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(MyKnobComponent),
      cv.Required(CONF_ENCODER): sensor.sensor_schema().extend(cv.polling_component_schema("100ms")),
    }
  )
  .extend(i2c.i2c_device_schema(CONF_I2C_ADDR))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    await light.register_light(var, config)

    sens = await sensor.new_sensor(config[CONF_ENCODER])
    cg.add(var.update_knob_value(sens))
