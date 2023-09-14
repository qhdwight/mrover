# Generated by Django 4.2.4 on 2023-09-07 01:20

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ("notes", "0001_initial"),
    ]

    operations = [
        migrations.CreateModel(
            name="Joystick",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("forward_back", models.FloatField()),
                ("left_right", models.FloatField()),
            ],
        ),
        migrations.DeleteModel(
            name="Note",
        ),
    ]
