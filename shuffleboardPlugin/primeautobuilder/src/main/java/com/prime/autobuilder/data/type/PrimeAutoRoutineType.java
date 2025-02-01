package com.prime.autobuilder.data.type;

import edu.wpi.first.shuffleboard.api.data.ComplexDataType;
import com.prime.autobuilder.data.PrimeAutoRoutineData;

import java.util.Map;
import java.util.function.Function;

/**
 * Represents data of the {@link PrimeAutoRoutineData} type.
 */
public final class PrimeAutoRoutineType extends ComplexDataType<PrimeAutoRoutineData> {

  public static final PrimeAutoRoutineType Instance = new PrimeAutoRoutineType();

  private PrimeAutoRoutineType() {
    super("PrimeAutoRoutine", PrimeAutoRoutineData.class);
  }

  @Override
  public Function<Map<String, Object>, PrimeAutoRoutineData> fromMap() {
    return PrimeAutoRoutineData::new;
  }

  @Override
  public PrimeAutoRoutineData getDefaultValue() {
    return new PrimeAutoRoutineData(new String[] {}, new String[] {}, new String[] {});
  }
}
