//! Decoding of NAOqi dynamic values into plain values and domain scalars.

use qi::value::{
    FromValue, FromValueError, IntoValue, Reflect, ToValue, Type, Value,
};

/// An owned NAOqi dynamic (`"m"`) value, as carried by event callbacks.
///
/// This is the payload type of the callback methods on the object the driver
/// serves to the robot; decoding into domain types happens in [`crate::qi::events`].
pub struct Raw(Value<'static>);

impl Raw {
    pub fn new(value: Value<'static>) -> Self {
        Self(value)
    }

    pub fn into_inner(self) -> Value<'static> {
        self.0
    }
}

impl std::fmt::Debug for Raw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl Reflect for Raw {
    fn ty() -> Option<Type> {
        None // dynamic "m"
    }
}

impl ToValue for Raw {
    fn to_value(&self) -> Value<'_> {
        self.0.to_value()
    }
}

impl<'a> IntoValue<'a> for Raw {
    fn into_value(self) -> Value<'a> {
        self.0.into_value()
    }
}

impl<'a> FromValue<'a> for Raw {
    fn from_value(value: Value<'a>) -> Result<Self, FromValueError> {
        Ok(Self(plain(value)))
    }
}

/// Wraps a value as NAOqi dynamic (`"m"`), as expected for ALValue parameters.
pub fn dynamic(value: impl IntoValue<'static>) -> Value<'static> {
    Value::Dynamic(Box::new(value.into_value()))
}

/// Strips dynamic wrappers at any depth and takes ownership.
pub fn plain(value: Value<'_>) -> Value<'static> {
    fn strip(value: Value<'_>) -> Value<'_> {
        match value {
            Value::Dynamic(inner) => strip(*inner),
            other => other,
        }
    }
    strip(value).into_owned()
}

/// Reads a float out of a numeric value of any width.
pub fn as_f32(value: &Value<'_>) -> Option<f32> {
    match plain(value.clone()) {
        Value::Float32(v) => Some(v.into_inner()),
        Value::Float64(v) => Some(v.into_inner() as f32),
        Value::Int8(v) => Some(f32::from(v)),
        Value::UInt8(v) => Some(f32::from(v)),
        Value::Int16(v) => Some(f32::from(v)),
        Value::UInt16(v) => Some(f32::from(v)),
        Value::Int32(v) => Some(v as f32),
        Value::UInt32(v) => Some(v as f32),
        Value::Int64(v) => Some(v as f32),
        Value::UInt64(v) => Some(v as f32),
        _ => None,
    }
}

/// Reads an integer out of a numeric value of any width.
pub fn as_i32(value: &Value<'_>) -> Option<i32> {
    match plain(value.clone()) {
        Value::Int8(v) => Some(i32::from(v)),
        Value::UInt8(v) => Some(i32::from(v)),
        Value::Int16(v) => Some(i32::from(v)),
        Value::UInt16(v) => Some(i32::from(v)),
        Value::Int32(v) => Some(v),
        Value::UInt32(v) => Some(v as i32),
        Value::Int64(v) => Some(v as i32),
        Value::UInt64(v) => Some(v as i32),
        Value::Float32(v) => Some(v.into_inner() as i32),
        Value::Float64(v) => Some(v.into_inner() as i32),
        _ => None,
    }
}

/// Reads a string value.
pub fn as_text(value: &Value<'_>) -> Option<String> {
    match plain(value.clone()) {
        Value::String(text) => text.as_str().map(str::to_owned),
        _ => None,
    }
}

/// Reads raw bytes.
pub fn as_bytes(value: &Value<'_>) -> Option<Vec<u8>> {
    match plain(value.clone()) {
        Value::Raw(bytes) => Some(bytes.into_owned()),
        Value::List(elements) => elements
            .iter()
            .map(|element| as_i32(element).map(|byte| byte as u8))
            .collect(),
        _ => None,
    }
}

/// Reads a list of floats out of a list or tuple of numbers.
pub fn as_f32s(value: &Value<'_>) -> Option<Vec<f32>> {
    match plain(value.clone()) {
        Value::List(elements) | Value::Tuple(elements) => {
            elements.iter().map(as_f32).collect::<Option<Vec<_>>>()
        }
        _ => None,
    }
}

/// Reads a list of strings out of a list or tuple.
pub fn as_texts(value: &Value<'_>) -> Option<Vec<String>> {
    match plain(value.clone()) {
        Value::List(elements) | Value::Tuple(elements) => {
            elements.iter().map(as_text).collect::<Option<Vec<_>>>()
        }
        _ => None,
    }
}

/// Adapts call arguments to the parameter signature advertised by the service.
///
/// NAOqi methods taking ALValue parameters expect them encoded as dynamic
/// values; plain parameters expect plain values. Proxies hand in plain values,
/// and this encodes each argument as the callee's signature demands.
pub fn adapt_args(args: Value<'static>, parameters: &Type) -> Value<'static> {
    let Type::Tuple(elements) = parameters else {
        return adapt_value(args, Some(parameters));
    };
    let expected = match elements {
        qi::value::ty::Tuple::Tuple(types) => types.clone(),
        qi::value::ty::Tuple::TupleStruct { elements, .. } => elements.clone(),
        qi::value::ty::Tuple::Struct { fields, .. } => {
            fields.iter().map(|field| field.ty.clone()).collect()
        }
    };
    let mut actual = match args {
        Value::Tuple(values) => values,
        Value::Unit => Vec::new(),
        single => vec![single],
    };
    actual.resize_with(expected.len(), || Value::Unit);
    Value::Tuple(
        actual
            .into_iter()
            .zip(&expected)
            .map(|(value, ty)| adapt_value(value, ty.as_ref()))
            .collect(),
    )
}

fn adapt_value(value: Value<'static>, expected: Option<&Type>) -> Value<'static> {
    match expected {
        None => Value::Dynamic(Box::new(value)),
        Some(_) => value,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_strips_nested_dynamic_wrappers() {
        let value = dynamic(dynamic(1.5f32));
        assert_eq!(plain(value), 1.5f32.into_value());
    }

    #[test]
    fn numeric_readers_accept_any_width() {
        assert_eq!(as_f32(&1.5f64.into_value()), Some(1.5));
        assert_eq!(as_f32(&3i32.into_value()), Some(3.0));
        assert_eq!(as_i32(&2.9f32.into_value()), Some(2));
        assert_eq!(as_i32(&dynamic(7i32.into_value())), Some(7));
        assert_eq!(as_f32(&"x".to_owned().into_value()), None);
    }

    #[test]
    fn list_readers_work_on_lists_and_tuples() {
        let floats = vec![1.0f32, 2.0, 3.0];
        assert_eq!(as_f32s(&floats.clone().into_value()), Some(floats));
        let tuple = Value::Tuple(vec![1i32.into_value(), 2i32.into_value()]);
        assert_eq!(as_f32s(&tuple), Some(vec![1.0, 2.0]));
        let texts = Value::List(vec!["a".to_owned().into_value(), dynamic("b".to_owned().into_value())]);
        assert_eq!(
            as_texts(&texts),
            Some(vec!["a".to_owned(), "b".to_owned()])
        );
    }

    #[test]
    fn raw_unwraps_dynamic_payloads() {
        let raw: Raw = Value::Dynamic(Box::new(vec![1u8, 2].into_value()))
            .cast_into()
            .unwrap();
        assert_eq!(raw.into_inner(), vec![1u8, 2].into_value());
    }

    #[test]
    fn adapt_args_wraps_only_dynamic_parameters() {
        let parameters = Type::Tuple(qi::value::ty::Tuple::Tuple(vec![
            Some(Type::String),
            None,
            Some(Type::Int32),
        ]));
        let args = Value::Tuple(vec![
            "key".to_owned().into_value(),
            vec![1.0f32].into_value(),
            7i32.into_value(),
        ]);
        let adapted = adapt_args(args, &parameters);
        let Value::Tuple(values) = adapted else {
            panic!("expected a tuple");
        };
        assert_eq!(values[0], "key".to_owned().into_value());
        assert!(matches!(values[1], Value::Dynamic(_)));
        assert_eq!(values[2], 7i32.into_value());
    }

    #[test]
    fn adapt_args_pads_missing_arguments() {
        let parameters = Type::Tuple(qi::value::ty::Tuple::Tuple(vec![None, None]));
        let Value::Tuple(values) = adapt_args(1.0f32.into_value(), &parameters) else {
            panic!("expected a tuple");
        };
        assert_eq!(values.len(), 2);
        for value in &values {
            assert!(matches!(value, Value::Dynamic(_)));
        }
    }
}
