package prime.control;

import java.util.function.DoubleSupplier;

public class SwerveControlSuppliers {

  public DoubleSupplier X;
  public DoubleSupplier Y;
  public DoubleSupplier Z;

  public SwerveControlSuppliers(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
    X = xSupplier;
    Y = ySupplier;
    Z = zSupplier;
  }
}
