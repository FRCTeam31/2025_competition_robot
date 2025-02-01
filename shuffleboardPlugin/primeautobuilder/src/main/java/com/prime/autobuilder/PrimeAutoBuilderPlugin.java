package com.prime.autobuilder;

import edu.wpi.first.shuffleboard.api.data.DataType;
import edu.wpi.first.shuffleboard.api.plugin.Description;
import edu.wpi.first.shuffleboard.api.plugin.Plugin;
import edu.wpi.first.shuffleboard.api.widget.ComponentType;
import edu.wpi.first.shuffleboard.api.widget.WidgetType;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.prime.autobuilder.data.type.PrimeAutoRoutineType;
import com.prime.autobuilder.widget.PrimeAutoBuilderWidget;

import java.util.List;
import java.util.Map;

/**
 * A plugin that provides a widget for building modular auto routines.
 */
@Description(group = "com.prime.autobuilder", name = "PrimeAutoBuilderPlugin", version = "0.1.0",
    summary = "A widget for building modular auto routines")
public final class PrimeAutoBuilderPlugin extends Plugin {

  @Override
  public List<DataType> getDataTypes() {
    return ImmutableList.of(PrimeAutoRoutineType.Instance);
  }

  @Override
  public List<ComponentType> getComponents() {
    return ImmutableList.of(WidgetType.forAnnotatedWidget(PrimeAutoBuilderWidget.class));
  }

  @Override
  public Map<DataType, ComponentType> getDefaultComponents() {
    return ImmutableMap.<DataType, ComponentType>builder().put(PrimeAutoRoutineType.Instance,
        WidgetType.forAnnotatedWidget(PrimeAutoBuilderWidget.class)).build();
  }
}
