package app.organicmaps.sdk.location.inertial;

import androidx.annotation.NonNull;

/**
 * Parses ELM327 responses to the OBD-II request "010D" (vehicle speed).
 */
public final class Elm327Parser
{
  public static final int NO_SPEED = -1;

  private Elm327Parser() {}

  /**
   * @param response everything the adapter has sent before the '>' prompt, e.g. "SEARCHING...\r41 0D 3C\r".
   * @return speed in km/h, or NO_SPEED if the response doesn't contain it (NO DATA, errors, etc.).
   */
  public static int parseSpeed(@NonNull String response)
  {
    // Spaces may be turned off by ATS0, several ECUs may answer on separate lines.
    for (String line : response.split("[\r\n]+"))
    {
      final String hex = line.replaceAll("\\s", "").toUpperCase();
      final int index = hex.indexOf("410D");
      if (index < 0 || hex.length() < index + 6)
        continue;
      try
      {
        return Integer.parseInt(hex.substring(index + 4, index + 6), 16);
      }
      catch (NumberFormatException e)
      {
        // Try the next line.
      }
    }
    return NO_SPEED;
  }
}
