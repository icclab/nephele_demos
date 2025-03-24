{{/* Define common labels for resources */}}
{{- define "nginx.labels" -}}
  app.kubernetes.io/name: nginx
  app.kubernetes.io/instance: {{ .Release.Name }}
  app.kubernetes.io/version: "1.0"
  app.kubernetes.io/managed-by: Helm
{{- end }}

{{/* Define the ConfigMap names */}}
{{- define "nginx.configMapName" -}}
  {{ .Release.Name }}-nginx-config
{{- end }}

{{/* Generate the full name of a resource (e.g., Service, Deployment) */}}
{{- define "nginx.fullname" -}}
  {{ .Release.Name }}-nginx
{{- end }}

