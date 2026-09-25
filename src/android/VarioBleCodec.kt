import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.UUID

/** Protocol codec; call from one serialized GATT callback/operation queue. */
object VarioBleIds {
    val service: UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    val rx: UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e")
    val tx: UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
    val telemetry: UUID = UUID.fromString("6e400004-b5a3-f393-e0a9-e50e24dcca9e")
    val gps: UUID = UUID.fromString("6e400005-b5a3-f393-e0a9-e50e24dcca9e")
    val health: UUID = UUID.fromString("6e400006-b5a3-f393-e0a9-e50e24dcca9e")
    val cccd: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
}

data class VarioTelemetry(
    val sequence: Int,
    val timeMs: Long,
    val altitudeM: Float,
    val climbRateMps: Float,
    val pressurePa: Float,
    val baroOk: Boolean,
    val imuOk: Boolean,
    val imuCalibrated: Boolean,
    val imuFusionActive: Boolean,
    val imuCalibrating: Boolean,
    val extendedFirmware: Boolean = false,
    val altitudeReferenceParity: Boolean = false
)

class VarioBleCodec {
    private val pending = ByteArrayOutputStream()

    fun reset() = pending.reset() // always call on disconnect/reconnect

    fun telemetry(bytes: ByteArray): VarioTelemetry {
        require(bytes.size == 20) { "Invalid telemetry length" }
        val b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        require(b.get().toInt() == 1) { "Unsupported protocol version" }
        val flags = b.get().toInt() and 0xff
        return VarioTelemetry(
            b.short.toInt() and 0xffff, b.int.toLong() and 0xffffffffL,
            b.float, b.float, b.float,
            flags and 1 != 0, flags and 2 != 0, flags and 4 != 0,
            flags and 8 != 0, flags and 16 != 0,
            flags and 32 != 0, flags and 64 != 0
        )
    }

    /** TX indications are fragments: decode UTF-8/JSON only after a full LF line. */
    fun responses(fragment: ByteArray): List<JSONObject> {
        val result = mutableListOf<JSONObject>()
        for (byte in fragment) {
            if (byte.toInt() == 10) {
                val line = pending.toByteArray().toString(Charsets.UTF_8)
                pending.reset()
                if (line.isNotBlank()) result += JSONObject(line)
            } else {
                if (pending.size() >= 32768) {
                    pending.reset()
                    error("Response too long; reconnect")
                }
                pending.write(byte.toInt() and 0xff)
            }
        }
        return result
    }

    /** Write each chunk with WRITE_TYPE_DEFAULT; wait for onCharacteristicWrite
     * before sending the next. Wait for the JSON reply before the next command. */
    fun command(id: Int, cmd: String, params: JSONObject = JSONObject()): List<ByteArray> {
        require(id > 0)
        val json = JSONObject().put("id", id).put("cmd", cmd).put("params", params)
        val bytes = json.toString().toByteArray(Charsets.UTF_8)
        require(bytes.size <= 511) { "Command exceeds firmware limit" }
        return (bytes + byteArrayOf(10)).toList().chunked(20).map { it.toByteArray() }
    }
}
